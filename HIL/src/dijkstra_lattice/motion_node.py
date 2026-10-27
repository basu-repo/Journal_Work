"""Tracking and planning adapter for a precompiled differential-drive lattice."""
import json
import math
import time
import threading
import numpy as np
from scipy.spatial import cKDTree
from tf2_ros import TransformException
from .grid import Grid
from .core import navigation_radius,physical_radius,rollout,swept_clear,wrap
from .lattice_node import LatticeNavigator
from .motion_graph import MotionGraph
from .primitives import load_library
from .directional_corridors import preferred_side


def remaining_lidar_clear(samples,points,radius,hard_radius):
    """Check the remaining primitive's physical swept corridor against live LiDAR."""
    samples=np.asarray(samples,dtype=float)
    points=np.asarray(points,dtype=float).reshape(-1,2)
    if not len(samples) or not len(points):return True,float('inf')
    distances=cKDTree(points).query(samples)[0]
    margin=max(np.linalg.norm(np.diff(samples,axis=0),axis=1),default=0.)/2
    closest=float(np.min(distances))
    if closest>radius+margin:return True,closest
    # If inflation grows around the current pose, permit a physically clear
    # escape only when every remaining sample moves away from the obstacle.
    escaping=(distances[0]<=radius+margin and closest>hard_radius+margin
              and np.all(np.diff(distances)>=-1e-3))
    return bool(escaping),closest

def committed_rollout_clear(grid,trial,active_samples,tolerance):
    """Accept a checked route despite stale inflation, never physical blockage."""
    trial=np.asarray(trial,float).reshape(-1,2)
    active=np.asarray(active_samples,float).reshape(-1,active_samples.shape[-1])[:,:2]
    if not len(trial) or not len(active):return False
    on_route=float(np.max(cKDTree(active).query(trial)[0]))<=tolerance
    return bool(on_route and grid.motion_clear(trial,grid.hard_radius))

class MotionNavigator(LatticeNavigator):
    planner_type=None
    algorithm='state_lattice_base'
    route_topic='/dijkstra_lattice/path'

    def make_planner(self):
        data=load_library(self.cfg['primitive_file'],self.robot,self.cfg)
        return self.planner_type().configure(data)

    def immediate_progress_segments(self,source,start,goal):
        """Optional model-specific fast path after the planning grid is ready."""
        return None

    def __init__(self,cfg,robot,output):
        super().__init__(cfg,robot,output)
        self.segments=[];self.active=None;self.active_samples=None
        self.active_goal=None;self.pending_goal=None
        self.primitive_started=0.;self.tracking_interrupts=0;self.last_block_log=-float('inf')
        self.pending_started=None;self.failed_subgoals=[]
        self.terminal_goal=None
        self.prefetch=None
        self.prefetch_started=0;self.prefetch_ready=0
        self.prefetch_accepted=0;self.prefetch_rejected=0
        self.recovery_rounds=0;self.recoveries_before_reset=0
        self.search_timeout_streak=0;self.search_round_robin_resets=0
        self.start_rejection_streak=0

    def reset_recovery_cycle(self,now,pose,local_pose):
        """Restart lattice navigation after three recoveries in GNSS denial.

        Sensor streams and the SLAM/EKF pose remain authoritative. Only stale
        navigation decisions are discarded, so the next tick plans afresh from
        the robot's current measured pose.
        """
        limit=int(self.cfg.get('recovery_round_robin_retries',0))
        if (limit<=0 or self.cfg.get('localization_mode')!='slam' or
                self.supervisor.recovery_count<limit):
            return False
        completed=self.supervisor.last_recovery_event in ('backup_finished','rotation_finished')
        failed=self.supervisor.state=='RECOVERY_FAILED'
        if not (completed or failed):return False
        count=self.supervisor.recovery_count
        self.recoveries_before_reset+=count;self.recovery_rounds+=1
        self.log('recovery_round_robin_reset',round=self.recovery_rounds,
                 recoveries=count,trigger=self.supervisor.last_recovery_event,
                 pose=[pose.x,pose.y,pose.yaw],gnss_requested=False)
        self._discard_prefetch('recovery_round_robin_reset')
        self.active=None;self.active_samples=None;self.active_goal=None
        self.segments=[];self.terminal_goal=None
        self.pending_goal=None;self.pending_started=None
        self.macro_goal=None;self.macro_subgoal=None;self.crop_center=None
        self.failed_subgoals=[];self.grid=None;self.path=[]
        self.planner=self.make_planner()
        self.route_reason='recovery_round_robin_reset'
        self.last_plan=-float('inf');self.previous=(0.,0.)
        self.progress_time=now;self.progress_pose=local_pose
        self.supervisor=type(self.supervisor)(self.robot)
        self.last_state=None
        self.publish_path()
        return True

    def invalidate_after_reanchor(self):
        self._discard_prefetch('gnss_reanchored')
        self.active=None;self.active_samples=None;self.active_goal=None
        self.segments=[];self.terminal_goal=None
        self.pending_goal=None;self.pending_started=None
        self.macro_subgoal=None;self.crop_center=None
        super().invalidate_after_reanchor()

    def _prefetch_enabled(self):
        # DijkstraLattice inherits this class but remains a separate baseline.
        return False

    def defer_periodic_gnss(self):
        """Finish a route while its localization estimate remains healthy."""
        return bool(self.localization_ok and
                    (self.active is not None or self.segments))

    def _discard_prefetch(self,reason):
        if self.prefetch is not None:
            self.prefetch['cancel'].set()
            self.log('lattice_prefetch_discarded',reason=reason)
            self.prefetch=None

    def _run_prefetch(self,job):
        """Search only the frozen grid; ROS state and commands stay on the main thread."""
        try:
            while not job['cancel'].is_set() and time.monotonic()<job['wall_deadline']:
                if job['planner'].compute(self.cfg['planning_max_expansions'],
                                          self.cfg['lattice_prefetch_time_slice_s']):
                    job['segments'],job['reason']=job['planner'].path()
                    break
                job['cancel'].wait(.002)
            if job['segments'] is None:
                job['segments']=[]
                job['reason']='cancelled' if job['cancel'].is_set() else 'wall_budget_exceeded'
        except Exception as exc:
            job['segments']=[];job['reason']='worker_error:'+type(exc).__name__
        finally:
            job['done']=True

    def _prepare_prefetch(self,goal,now):
        if (not self._prefetch_enabled() or self.prefetch is not None or
                self.active is None or not self.segments or
                len(self.segments)>self.cfg['lattice_prefetch_remaining_segments'] or
                self.map is None or self.grid is None):
            return
        end=self.segments[-1][1]
        if len(self.segments)==1 and self.active_samples is not None:
            predicted=tuple(map(float,self.active_samples[-1,:2]))
            predicted_yaw=float(self.active_samples[-1,2])
        else:
            predicted=self.grid.xy(end[:2])
            predicted_yaw=(end[2]*2*math.pi/self.cfg['lattice_headings']+
                           math.atan2(self.grid.rot[1,0],self.grid.rot[0,0]))
        # Make an independent sensor-grid snapshot. Do not move the active
        # controller's crop while planning from the predicted end position.
        saved_center=self.crop_center
        try:
            self.crop_center=tuple(predicted)
            local_map=self.local_map(self.map,predicted)
        finally:
            self.crop_center=saved_center
        distance=math.dist(predicted,goal)
        scale=min(1.,self.cfg['lattice_horizon_m']/max(distance,1e-6))
        seed=(predicted[0]+(goal[0]-predicted[0])*scale,
              predicted[1]+(goal[1]-predicted[1])*scale)
        try:
            free_scan=self.scan_free_evidence()
            occupied_scan=self.obstacles()
            grid=Grid(local_map,predicted,seed,navigation_radius(self.sigma,self.robot,self.cfg),
                      self.cfg,hard_radius=physical_radius(self.robot,self.cfg),
                      free_scan=free_scan,occupied_scan=occupied_scan)
        except (TransformException,ValueError):
            return
        grid.allow_clear_start(predicted)
        subgoal=self.virtual_goal(grid,predicted,goal)
        if subgoal is None:return
        yaw=math.atan2(grid.rot[1,0],grid.rot[0,0])
        heading=round((predicted_yaw-yaw)/(2*math.pi/self.cfg['lattice_headings']))%self.cfg['lattice_headings']
        source=(*grid.cell(predicted),heading)
        target=grid.cell(subgoal)
        if not grid.free(source[:2]) or not grid.free(target):return
        planner=MotionGraph().configure(self.planner.library)
        planner.update(grid,source,target,predicted,subgoal)
        self.prefetch=dict(goal=tuple(goal),start=tuple(predicted),yaw=predicted_yaw,
                           subgoal=tuple(subgoal),planner=planner,segments=None,
                           started=now,done=False,reported=False,reason=None,
                           cancel=threading.Event(),
                           wall_deadline=time.monotonic()+self.cfg['lattice_prefetch_max_wall_s'])
        self.prefetch_started+=1
        self.log('lattice_prefetch_started',predicted_start=predicted,subgoal=subgoal)
        threading.Thread(target=self._run_prefetch,args=(self.prefetch,),daemon=True).start()

    def _advance_prefetch(self,goal,now):
        if not self._prefetch_enabled():return
        job=self.prefetch
        if job is not None and job['goal']!=tuple(goal):
            self._discard_prefetch('goal_changed');job=None
        if job is None:
            self._prepare_prefetch(goal,now)
            job=self.prefetch
        if job is None:return
        if now-job['started']>self.cfg['lattice_prefetch_max_age_s']:
            self._discard_prefetch('expired');return
        if job['done'] and not job['reported']:
            job['reported']=True
            if job['segments']:
                self.prefetch_ready+=1
                self.log('lattice_prefetch_ready',segments=len(job['segments']),
                         expansions=job['planner'].expansions,reason=job['reason'])
            else:
                self.log('lattice_prefetch_no_route',reason=job['reason'])

    def _accept_prefetch(self,pose,goal,now):
        job=self.prefetch
        if not self._prefetch_enabled() or job is None:return False
        self.prefetch=None
        job['cancel'].set()
        reason=None
        position_error=math.dist((pose.x,pose.y),job['start'])
        heading_error=abs(wrap(pose.yaw-job['yaw']))
        if not job['done'] or not job['segments']:reason='unfinished'
        elif job['goal']!=tuple(goal):reason='goal_changed'
        elif now-job['started']>self.cfg['lattice_prefetch_max_age_s']:reason='expired'
        elif position_error>self.cfg['lattice_prefetch_position_tolerance_m']:
            reason='position_changed'
        elif heading_error>self.cfg['lattice_prefetch_heading_tolerance_rad']:
            reason='heading_changed'
        if reason is not None:
            self.prefetch_rejected+=1
            self.log('lattice_prefetch_rejected',reason=reason,
                     position_error=position_error,heading_error=heading_error)
            return False
        saved_center=self.crop_center
        try:
            self.crop_center=tuple(job['start'])
            local_map=self.local_map(self.map,(pose.x,pose.y))
        finally:
            self.crop_center=saved_center
        try:
            free_scan=self.scan_free_evidence()
            points=self.obstacles()
            fresh=Grid(local_map,(pose.x,pose.y),job['subgoal'],
                       navigation_radius(self.sigma,self.robot,self.cfg),self.cfg,
                       hard_radius=physical_radius(self.robot,self.cfg),
                       free_scan=free_scan,occupied_scan=points)
        except (TransformException,ValueError):
            reason='fresh_inputs_unavailable'
        else:
            fresh.allow_clear_start((pose.x,pose.y))
            if not job['planner'].same_geometry(fresh):reason='map_geometry_changed'
            else:
                for state,_,primitive in job['segments']:
                    swept=job['planner'].world_samples(state,primitive)
                    radius=fresh.hard_radius if primitive['kind']=='rotate' else fresh.radius
                    if not fresh.motion_clear(swept,radius):
                        reason='fresh_map_blocked';break
        if reason is not None:
            self.prefetch_rejected+=1;self.log('lattice_prefetch_rejected',reason=reason)
            return False
        old_planner,old_grid=self.planner,self.grid
        self.planner=job['planner'];self.planner.grid=fresh;self.grid=fresh
        self.segments=list(job['segments']);self.macro_subgoal=job['subgoal']
        self.pending_goal=None;self.pending_started=None
        self.path=self._route_points(self.segments);self.route_reason=self.algorithm
        self._activate(pose,goal,now)
        if self.active is None or not self._active_valid(pose,points):
            self.planner=old_planner;self.grid=old_grid
            self.active=None;self.active_samples=None;self.segments=[];self.path=[]
            self.prefetch_rejected+=1
            self.log('lattice_prefetch_rejected',reason='fresh_first_primitive_blocked')
            return False
        self.prefetch_accepted+=1;self.route_sequence+=1;self.publish_path()
        self.log('lattice_prefetch_accepted',segments=len(self.segments))
        return True

    def _route_points(self,segments):
        points=[]
        for a,b,primitive in segments:
            for x,y in self.planner.world_samples(a,primitive)[::max(1,len(primitive['samples'])//12)]:
                xy=(float(x),float(y))
                if not points or math.dist(points[-1],xy)>1e-5:points.append(xy)
        if segments:
            end=segments[-1][1]
            xy=self.grid.xy(end[:2])
            if not points or math.dist(points[-1],xy)>1e-5:points.append(xy)
        return points

    def _activate(self,pose,goal,now):
        if not self.segments:return
        state,_,primitive=self.segments[0]
        nominal=state[2]*2*math.pi/self.cfg['lattice_headings']+math.atan2(self.grid.rot[1,0],self.grid.rot[0,0])
        residual=wrap(pose.yaw-nominal)
        # Anchor the precompiled curve to the measured pose and heading.
        samples=np.asarray(primitive['samples'],float)
        c,s=math.cos(residual),math.sin(residual)
        rotation=np.array([[c,-s],[s,c]])
        offset=samples[:,:2]@self.grid.rot.T@rotation.T
        xy=np.asarray((pose.x,pose.y))+offset
        yaw=samples[:,2]+math.atan2(self.grid.rot[1,0],self.grid.rot[0,0])+residual
        radius=self.grid.hard_radius if primitive['kind']=='rotate' else self.grid.radius
        if not self.grid.motion_clear(xy,radius):
            self.start_rejection_streak+=1
            self.log('primitive_rejected',reason='adapted_sweep_blocked',
                     primitive_id=primitive['id'],
                     consecutive_start_rejections=self.start_rejection_streak)
            self.active=None;self.active_samples=None;self.segments=[]
            self.path=[];self.route_reason='primitive_start_blocked'
            if self.algorithm=='dstar_state_lattice' and self.macro_subgoal is not None:
                # DijkstraLattice already discards this failed macro target.
                # Original D* must do the same, or it searches repeatedly
                # toward a target whose first measured-pose sweep is blocked.
                self.failed_subgoals.append(self.macro_subgoal)
                self.failed_subgoals=self.failed_subgoals[-8:]
                self.macro_subgoal=None
            return
        self.active=(state,primitive)
        self.start_rejection_streak=0
        # A usable motion has been acquired, so the planning-pause watchdog is
        # satisfied. Only consecutive searches with no executable primitive
        # count toward its round-robin reset.
        self.search_timeout_streak=0
        self.active_samples=np.column_stack((xy,yaw,samples[:,3:]))
        self.active_goal=tuple(goal)
        self.primitive_started=now
        self.progress_time=now
        self.log('primitive_started',primitive_id=primitive['id'],kind=primitive['kind'],
                 nominal_length=primitive['length'],nominal_duration=primitive['duration'],
                 initial_heading_error=residual)

    def _active_valid(self,pose,points):
        if self.active_samples is None:return False
        samples=self.active_samples
        delta=samples[:,:2]-np.array([pose.x,pose.y])
        nearest=int(np.argmin(np.linalg.norm(delta,axis=1)))
        if np.linalg.norm(delta[nearest])>self.cfg['lattice_tracking_deviation_m']:
            self.log('primitive_interrupted',reason='cross_track_error',
                     primitive_id=self.active[1]['id'],deviation=float(np.linalg.norm(delta[nearest])))
            return False
        if self.active[1]['kind']=='curve' and len(points):
            horizon=samples[nearest:,:2]
            if len(horizon):
                # The route was already planned with covariance-scaled inflation.
                # During execution, use fresh LiDAR only to stop for an object in
                # the robot's physical swept corridor.  Reapplying localization
                # inflation here makes harmless side returns cancel a valid route.
                radius=physical_radius(self.robot,self.cfg)
                clear,distance=remaining_lidar_clear(
                    horizon,points,radius,physical_radius(self.robot,self.cfg))
                if not clear:
                    self.log('primitive_interrupted',reason='fresh_lidar_obstacle',
                             primitive_id=self.active[1]['id'],clearance=float(distance),
                             required_clearance=float(radius))
                    return False
        return True

    def _finish_active(self,pose,now):
        if self.active is None:return False
        end=self.active_samples[-1]
        error=math.dist((pose.x,pose.y),end[:2])
        heading=abs(wrap(pose.yaw-end[2]))
        limit=(self.cfg['lattice_rotation_tolerance_rad'] if self.active[1]['kind']=='rotate'
               else self.cfg['lattice_heading_tolerance_rad'])
        if error<=self.cfg['lattice_endpoint_tolerance_m'] and heading<=limit:
            self.log('primitive_completed',primitive_id=self.active[1]['id'],
                     position_error=error,heading_error=heading,
                     elapsed=now-self.primitive_started)
            goal=self.active_goal
            self.active=None;self.active_samples=None;self.last_plan=-float('inf')
            if self.segments:self.segments=self.segments[1:]
            if self.segments:
                self._activate(pose,goal,now)
                if self.active is not None:
                    try:points=self.obstacles()
                    except TransformException:points=np.empty((0,2))
                    if self._active_valid(pose,points):
                        self.path=self._route_points(self.segments)
                        self.publish_path()
                        self.log('route_primitive_continued',remaining=len(self.segments))
                        return True
                self.active=None;self.active_samples=None
                self.segments=[];self.path=[]
            elif self.gnss and not self.target_gnss:
                # A local lattice route is one commitment. Refresh localization
                # at its boundary instead of interrupting its individual pieces.
                self._discard_prefetch('committed_route_completed')
                self.path=[];self.route_reason='committed_route_completed'
                self.publish_path()
                self.enter_gnss_acquire(now,'committed_route_completed')
                return True
            elif self._accept_prefetch(pose,goal,now):
                return True
            return True
        if now-self.primitive_started>max(self.cfg['lattice_primitive_timeout_s'],
                                           self.active[1]['duration']*self.cfg['lattice_duration_multiplier']):
            self.log('primitive_interrupted',reason='tracking_timeout',
                     primitive_id=self.active[1]['id'],position_error=error,heading_error=heading)
            self.tracking_interrupts+=1
            self.active=None;self.active_samples=None;self.last_plan=-float('inf')
            return True
        return False

    def virtual_goal(self,grid,start,goal):
        distance=math.dist(start,goal)
        horizon=self.cfg['lattice_horizon_m']
        if distance<=horizon and grid.free(grid.cell(goal)):return goal
        bearing=math.atan2(goal[1]-start[1],goal[0]-start[0])
        # Fresh 360-degree LiDAR breaks ties between feasible corridors.
        # The inflated grid still decides which paths are safe.
        assessment=self.assess_directions(self.pose,grid,heading=bearing) if self.pose else None
        side=preferred_side(assessment,self.cfg['direction_probe_m'])
        candidates=[]
        for offset in np.linspace(-math.pi,math.pi,49):
            for radius in (horizon,horizon*.75,horizon*.5,horizon*.4):
                point=(start[0]+radius*math.cos(bearing+offset),
                       start[1]+radius*math.sin(bearing+offset))
                cell=grid.cell(point)
                if not grid.free(cell):continue
                center=grid.xy(cell)
                if any(math.dist(center,old)<1.25 for old in self.failed_subgoals):continue
                if not grid.line_clear(start,center):continue
                clearance=float(grid.distance[cell[1],cell[0]])
                side_penalty=0.6 if ((side=='left' and offset < -0.1) or
                                     (side=='right' and offset > 0.1)) else 0.
                remaining=math.dist(center,goal)
                # Safety is already enforced by free() and line_clear(). Among
                # safe candidates, make progress toward the upcoming waypoint
                # the primary objective. Direction and clearance only break
                # ties; they must not select a farther target in open space.
                candidates.append((remaining,side_penalty,abs(offset),
                                   -clearance,center))
        if not candidates:
            self.failed_subgoals=[]
            return None
        minimum_progress=float(self.cfg.get('lattice_min_goal_progress_m',.2))
        progressing=[item for item in candidates
                     if item[0]<=distance-minimum_progress]
        pool=progressing if progressing else candidates
        return min(pool,key=lambda item:item[:4])[4]

    def global_plan(self,pose,goal,now):
        # The base mission tick calls planning before the supervisor step.
        # A pending D* search would make that tick return before recovery
        # receives its reverse/rotation command. Suspend search completely
        # until the supervisor has finished its maneuver or waypoint handoff.
        if self.supervisor.state not in ('GOTO_WAYPOINT','OBSTACLE_AVOID'):
            self._discard_prefetch('supervisor_interrupted')
            self.active=None;self.active_samples=None;self.active_goal=None
            self.terminal_goal=None;self.segments=[];self.path=[]
            self.pending_goal=None;self.pending_started=None
            self.route_reason='supervisor_suspended'
            self.last_plan=-float('inf')
            return
        # Lattice nodes are quantized, while a waypoint is a continuous point.
        # Use a short checked terminal connector once the exact target lies
        # inside a clear local corridor; the live LiDAR safety gate remains on.
        distance=math.dist((pose.x,pose.y),goal)
        if (distance<=self.cfg['lattice_terminal_activation_m'] and
                self.grid is not None and self.grid.line_clear((pose.x,pose.y),goal)):
            self._discard_prefetch('terminal_connector')
            if self.terminal_goal!=tuple(goal):
                self.log('terminal_connector_started',distance=distance)
            self.terminal_goal=tuple(goal)
            self.active=None;self.active_samples=None;self.segments=[]
            self.pending_goal=None;self.pending_started=None
            self.path=[(pose.x,pose.y),tuple(goal)]
            self.route_reason='terminal_connector'
            self.publish_path()
            return
        self.terminal_goal=None
        if self.active is not None:
            if (self.active_goal!=tuple(goal) or self.planner.grid is None or
                    self.supervisor.state in ('RECOVERY_BACKUP','RECOVERY_ROTATE','RECOVERY_FAILED')):
                self._discard_prefetch('active_route_changed')
                self.active=None;self.active_samples=None;self.last_plan=-float('inf')
            else:
                self._advance_prefetch(goal,now)
                completed=self._finish_active(pose,now)
                if completed:return
                if not completed:
                    try:points=self.obstacles()
                    except TransformException:points=np.empty((0,2))
                    if self._active_valid(pose,points):return
                    self._discard_prefetch('active_route_interrupted')
                    self.active=None;self.active_samples=None;self.tracking_interrupts+=1
                    self.last_plan=-float('inf');self.path=[];self.route_reason='primitive_interrupted'
                    return

        if (self.route_reason=='incremental_search_timeout' and
                self.pending_goal==tuple(goal) and self.planner.grid is not None):
            if (self.pending_started is not None and
                    now-self.pending_started>self.cfg['lattice_search_budget_s']):
                self.log('lattice_search_budget_exceeded',subgoal=self.macro_subgoal,
                         expansions=self.planner.expansions)
                self.search_timeout_streak+=1
                if self.macro_subgoal is not None:
                    self.failed_subgoals.append(self.macro_subgoal)
                    self.failed_subgoals=self.failed_subgoals[-8:]
                self.macro_subgoal=None;self.pending_goal=None;self.pending_started=None
                self.planner=self.make_planner();self.path=[];self.segments=[]
                self.route_reason='search_budget_exceeded';self.last_plan=-float('inf')
                # Computation alone is not evidence of a physically blocked
                # robot.  This applies to Dijkstra as well as D*; otherwise a
                # search timeout is misclassified as a driving stall and
                # triggers a needless three-metre recovery backup.
                self.supervisor.blocked_since=None
                limit=int(self.cfg.get('lattice_search_round_robin_retries',0))
                if limit>0 and self.search_timeout_streak>=limit:
                    self.search_round_robin_resets+=1
                    self.log('lattice_search_round_robin_reset',
                             round=self.search_round_robin_resets,
                             consecutive_timeouts=self.search_timeout_streak,
                             pose=[pose.x,pose.y,pose.yaw],gnss_requested=False)
                    self.failed_subgoals=[];self.macro_goal=None
                    self.crop_center=None;self.grid=None
                    self.planner=self.make_planner()
                    self.search_timeout_streak=0
                    self.route_reason='search_round_robin_reset'
                return
            self.last_plan=now
            if not self.planner.compute(self.cfg['planning_max_expansions'],self.cfg['planning_time_limit']):
                self.search_pending_count+=1
                return
            self.grid=self.planner.grid
            self.segments,self.route_reason=self.planner.path()
            self.path=self._route_points(self.segments)
            self.pending_goal=None;self.pending_started=None;self.route_sequence+=1;self.publish_path()
            if self.segments:self._activate(pose,goal,now)
            return

        if now-self.last_plan<self.cfg['replan_interval'] and self.path:return
        self.last_plan=now;self.global_checks+=1
        start=(pose.x,pose.y)
        try:free_scan=self.scan_free_evidence()
        except TransformException:free_scan=None
        try:occupied_scan=self.obstacles()
        except TransformException:occupied_scan=None
        distance=math.dist(start,goal)
        scale=min(1.,self.cfg['lattice_horizon_m']/max(distance,1e-6))
        seed=(start[0]+(goal[0]-start[0])*scale,start[1]+(goal[1]-start[1])*scale)
        try:
            grid=Grid(self.local_map(self.map,start),start,seed,navigation_radius(self.sigma,self.robot,self.cfg),
                      self.cfg,hard_radius=physical_radius(self.robot,self.cfg),
                      free_scan=free_scan,occupied_scan=occupied_scan)
        except ValueError as exc:
            self.grid=None;self.path=[];self.segments=[];self.route_reason=str(exc);self.publish_path();return
        self.grid=grid;grid.allow_clear_start(start)
        map_yaw=math.atan2(grid.rot[1,0],grid.rot[0,0])
        heading=round((pose.yaw-map_yaw)/(2*math.pi/self.cfg['lattice_headings']))%self.cfg['lattice_headings']
        source=(*grid.cell(start),heading)
        if self.macro_goal!=tuple(goal):
            self.macro_goal=tuple(goal);self.macro_subgoal=None
        # Keep the selected target when the direct sightline becomes blocked.
        # D* can repair the route around the new obstacle; replacing the target
        # here would discard its search tree on every sensor update.
        if (self.macro_subgoal is not None and math.dist(start,self.macro_subgoal)>1.
                and grid.free(grid.cell(self.macro_subgoal))):
            subgoal=self.macro_subgoal
        else:
            subgoal=self.virtual_goal(grid,start,goal)
            self.macro_subgoal=subgoal
        if subgoal is None:
            self.path=[];self.segments=[];self.route_reason='no_local_subgoal';self.publish_path();return
        target=grid.cell(subgoal)
        update=self.planner.update(grid,source,target,start,subgoal)
        self.route_searches+=1
        if not grid.free(target):self.segments=[];self.route_reason='goal_occupied'
        elif not grid.free(source[:2]):self.segments=[];self.route_reason='start_occupied'
        elif (direct:=self.immediate_progress_segments(source,start,goal)):
            # A model may select an already collision-checked lattice edge
            # before launching a multi-second graph search.  It remains a
            # kinematically valid primitive and is checked again after being
            # anchored to the measured pose by _activate().
            self.segments=direct;self.route_reason=self.algorithm+'_direct_progress'
            self.pending_goal=None;self.pending_started=None
        elif not self.planner.compute(self.cfg['planning_max_expansions'],self.cfg['planning_time_limit']):
            self.segments=[];self.route_reason='incremental_search_timeout';self.search_pending_count+=1
            self.pending_goal=tuple(goal);self.pending_started=now
            self.log('lattice_search_pending',update=update,expansions=self.planner.expansions)
        else:
            self.segments,self.route_reason=self.planner.path();self.pending_goal=None;self.pending_started=None
        self.path=self._route_points(self.segments)
        self.route_sequence+=1;self.publish_path()
        if self.segments:self._activate(pose,goal,now)

    def local_target(self,pose,lookahead=None):
        if self.terminal_goal is not None:return self.terminal_goal
        if self.active_samples is None:return None
        samples=self.active_samples
        nearest=int(np.argmin(np.linalg.norm(samples[:,:2]-np.array([pose.x,pose.y]),axis=1)))
        if self.active[1]['kind']=='rotate':
            angle=samples[-1,2]
            return (pose.x+math.cos(angle),pose.y+math.sin(angle))
        distance=samples[:,5]
        desired=distance[nearest]+self.cfg['lattice_tracking_lookahead_m']
        index=min(len(samples)-1,int(np.searchsorted(distance,desired)))
        return tuple(map(float,samples[index,:2]))

    def braking_distance(self,remaining,points):
        """Distance available to stop before the current primitive ends."""
        if (self._prefetch_enabled() and self.active is not None and
                self.active_samples is not None and len(self.segments)>=2 and self.grid is not None):
            state,_,primitive=self.segments[1]
            if primitive['kind']=='curve':
                nominal=(state[2]*2*math.pi/self.cfg['lattice_headings']+
                         math.atan2(self.grid.rot[1,0],self.grid.rot[0,0]))
                end=self.active_samples[-1]
                residual=wrap(end[2]-nominal)
                samples=np.asarray(primitive['samples'],float)
                c,s=math.cos(residual),math.sin(residual)
                xy=end[:2]+samples[:,:2]@self.grid.rot.T@np.array([[c,-s],[s,c]]).T
                if (self.grid.motion_clear(xy,self.grid.radius) and
                        (not len(points) or np.min(cKDTree(points).query(xy)[0])>
                         physical_radius(self.robot,self.cfg))):
                    return remaining+min(float(primitive['length']),
                                         self.cfg['lattice_handoff_lookahead_m'])
        return remaining

    def propose_command(self,pose,target,points):
        if self.terminal_goal is not None:
            dx=self.terminal_goal[0]-pose.x;dy=self.terminal_goal[1]-pose.y
            distance=math.hypot(dx,dy)
            error=wrap(math.atan2(dy,dx)-pose.yaw)
            dt=1/self.cfg['controller_frequency']
            dv=self.robot['linear_acceleration']*dt
            dw=self.robot['angular_acceleration']*dt
            if abs(error)>self.cfg['lattice_terminal_heading_tolerance_rad']:
                desired_v=0.
            else:
                arrival_radius=max(0.05,self.cfg['goal_tolerance']-
                                   self.cfg['arrival_reserve'])
                desired_v=min(self.cfg['lattice_terminal_speed'],
                              math.sqrt(2*self.robot['linear_acceleration']*
                                        max(0.,distance-arrival_radius*.5)))
            desired_w=max(-self.robot['max_angular_speed'],
                          min(self.robot['max_angular_speed'],1.5*error))
            v=max(0.,min(desired_v,self.previous[0]+dv))
            if self.previous[0]>desired_v:v=max(desired_v,self.previous[0]-dv)
            w=max(self.previous[1]-dw,min(self.previous[1]+dw,desired_w))
            trial=rollout(pose,v,w,self.cfg['prediction_dt'],self.cfg['prediction_steps'])
            lidar_clear,_=swept_clear(pose,v,w,points,self.robot,self.cfg)
            if not (self.trajectory_clear(trial[:,:2]) and lidar_clear):
                self.log('terminal_connector_blocked',distance=distance)
                self.terminal_goal=None;self.path=[];self.last_plan=-float('inf')
                return 0.,0.,'tracking'
            return float(v),float(w),'tracking'
        if self.active_samples is None:
            # A planner budget expiration means this subgoal was computationally
            # unproductive. It is not evidence that the robot is physically
            # trapped, so the supervisor must not start backup recovery while
            # the next 360-degree subgoal is being selected.
            if self.route_reason=='search_budget_exceeded':
                return 0.,0.,'planning'
            return 0.,0.,'blocked'
        samples=self.active_samples
        nearest=int(np.argmin(np.linalg.norm(samples[:,:2]-np.array([pose.x,pose.y]),axis=1)))
        dt=1/self.cfg['controller_frequency']
        if self.active[1]['kind']=='rotate':
            error=wrap(samples[-1,2]-pose.yaw)
            if abs(error)<self.cfg['lattice_rotation_tolerance_rad']:return 0.,0.,'tracking'
            desired=max(-self.robot['max_angular_speed'],min(self.robot['max_angular_speed'],1.5*error))
            dw=self.robot['angular_acceleration']*dt
            w=max(self.previous[1]-dw,min(self.previous[1]+dw,desired))
            v=0.
        else:
            distance=samples[:,5]
            ahead=min(len(samples)-1,int(np.searchsorted(
                distance,distance[nearest]+self.cfg['lattice_tracking_lookahead_m'])))
            target_xy=samples[ahead,:2]
            dx,dy=target_xy[0]-pose.x,target_xy[1]-pose.y
            forward=math.cos(pose.yaw)*dx+math.sin(pose.yaw)*dy
            lateral=-math.sin(pose.yaw)*dx+math.cos(pose.yaw)*dy
            look=max(math.hypot(dx,dy),.25)
            curvature=2*lateral/look**2
            yaw_error=wrap(samples[ahead,2]-pose.yaw)
            remaining=max(0.,float(distance[-1]-distance[nearest]))
            brake_distance=self.braking_distance(remaining,points)
            brake_speed=math.sqrt(2*self.robot['linear_acceleration']*
                                  max(0.,brake_distance-self.cfg['lattice_endpoint_tolerance_m']*.5))
            curve=max(abs(float(samples[ahead,3])),1e-6)
            curve_speed=min(self.robot['max_angular_speed']/curve,
                            math.sqrt(self.cfg['lattice_lateral_acceleration']/curve))
            vmax=min(self.robot['max_linear_speed'],self.cfg['lattice_cruise_speed'],
                     curve_speed,max(.12,brake_speed))
            if abs(yaw_error)>self.cfg['lattice_max_tracking_heading_error_rad']:
                vmax=0.
            desired_w=max(-self.robot['max_angular_speed'],
                          min(self.robot['max_angular_speed'],vmax*curvature+.4*yaw_error))
            dv=self.robot['linear_acceleration']*dt
            dw=self.robot['angular_acceleration']*dt
            v=max(0.,min(vmax,self.previous[0]+dv))
            if self.previous[0]>vmax:v=max(vmax,self.previous[0]-dv)
            w=max(self.previous[1]-dw,min(self.previous[1]+dw,desired_w))
        trial=rollout(pose,v,w,self.cfg['prediction_dt'],self.cfg['prediction_steps'])
        grid_clear=self.trajectory_clear(trial[:,:2])
        lidar_clear,nearest=swept_clear(pose,v,w,points,self.robot,self.cfg)
        committed=(not grid_clear and lidar_clear and
                   committed_rollout_clear(self.grid,trial[:,:2],self.active_samples,
                                           self.cfg['lattice_tracking_deviation_m']))
        if committed:
            grid_clear=True
            if self.now()-self.last_block_log>=1.:
                self.last_block_log=self.now()
                self.log('committed_primitive_continued',primitive_id=self.active[1]['id'],
                         reason='lidar_and_physical_clear',nearest_lidar=float(nearest))
        if not (grid_clear and lidar_clear):
            if self.now()-self.last_block_log>=1.:
                self.last_block_log=self.now()
                self.log('primitive_command_blocked',primitive_id=self.active[1]['id'],
                         grid_clear=bool(grid_clear),lidar_clear=bool(lidar_clear),
                         nearest_lidar=float(nearest),pose=[pose.x,pose.y,pose.yaw],
                         candidate=[float(v),float(w)],trial_end=trial[-1,:2].tolist(),
                         grid_source_cell=[int(c) for c in self.grid.cell((pose.x,pose.y))])
            self.active=None;self.active_samples=None;self.segments=[]
            self.path=[];self.last_plan=-float('inf')
            self.route_reason='primitive_command_blocked'
            return 0.,0.,'tracking'
        return float(v),float(w),'tracking'

    def summary(self):
        super().summary()
        p=self.output/'summary.json';data=json.loads(p.read_text())
        data['planning']['algorithm']=self.algorithm
        data['planning']['lattice_primitives']=['precompiled_curves','in_place_rotations']
        data['planning']['primitive_library_count']=len(self.planner.library['primitives'])
        data['planning']['tracking_interrupts']=self.tracking_interrupts
        data['planning']['recovery_round_robin']=dict(
            rounds=self.recovery_rounds,
            recoveries_before_reset=self.recoveries_before_reset)
        data['planning']['search_round_robin']=dict(
            resets=self.search_round_robin_resets,
            active_timeout_streak=self.search_timeout_streak)
        if self._prefetch_enabled():
            data['planning']['prefetch']=dict(started=self.prefetch_started,ready=self.prefetch_ready,
                                              accepted=self.prefetch_accepted,rejected=self.prefetch_rejected)
        p.write_text(json.dumps(data,indent=2))
