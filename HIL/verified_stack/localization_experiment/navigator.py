"""Opt-in experimental global guidance, existing Dijkstra lattice execution."""
import sys,math,json,time
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from DijkstraLattice.node import DijkstraNavigator
from Dstar.node import main
from rclpy.time import Time
from tf2_ros import TransformException
from Dstar.core import physical_radius,navigation_radius
from Dstar.route import Grid
from exploration_memory.core import MapGrid
from localization_experiment.global_guidance import Guidance
from localization_experiment.scan_evidence import supplement_free
from localization_experiment.horizon_fallback import HorizonFallback
HEADING_GUIDANCE='--heading-guidance' in sys.argv
if HEADING_GUIDANCE:
    sys.argv.remove('--heading-guidance')
    from localization_experiment.heading_guidance import Guidance
NEARBY_GUIDANCE='--nearby-guidance' in sys.argv
if NEARBY_GUIDANCE:
    sys.argv.remove('--nearby-guidance')
    from localization_experiment.nearby_guidance import Guidance
STRAIGHT_ONLY='--straight-only' in sys.argv
if STRAIGHT_ONLY:sys.argv.remove('--straight-only')
BOUNDARY_FOLLOW='--boundary-follow' in sys.argv
if BOUNDARY_FOLLOW:sys.argv.remove('--boundary-follow')
ADAPTIVE_HORIZON='--adaptive-horizon' in sys.argv
if ADAPTIVE_HORIZON:sys.argv.remove('--adaptive-horizon')
MAP_CORRECTIONS='--map-corrections' in sys.argv
if MAP_CORRECTIONS:sys.argv.remove('--map-corrections')
FULL_GUIDANCE_OBSERVATION='--full-guidance-observation' in sys.argv
if FULL_GUIDANCE_OBSERVATION:sys.argv.remove('--full-guidance-observation')
class Explorer(DijkstraNavigator):
    def __init__(self,*args):
        super().__init__(*args)
        if STRAIGHT_ONLY:
            self.cfg['lattice_rotation_tolerance_rad']=.025
        self.guidance=Guidance(boundary_follow=BOUNDARY_FOLLOW);self.guidance_cfg=yaml.safe_load((ROOT/'exploration_memory/settings.yaml').read_text())
        self.guidance.observe_full_state=FULL_GUIDANCE_OBSERVATION
        self.guidance_audit=(self.output/'global_guidance.jsonl').open('w')
        self.full_guidance_observer=None;self.full_guidance_decision_id=0
        if FULL_GUIDANCE_OBSERVATION:
            from localization_experiment.full_guidance_observer import FullGuidanceObserver
            self.full_guidance_observer=FullGuidanceObserver(self.output)
        self.map_correction_reference=None;self.map_correction_hold=None;self.horizon_fallback=HorizonFallback()
    def _activate(self,pose,goal,now):
        if STRAIGHT_ONLY and self.segments and self.segments[0][2]['kind']=='curve':
            from localization_experiment.straight_primitives import alignment_primitive
            state=self.segments[0][0]
            map_yaw=math.atan2(self.grid.rot[1,0],self.grid.rot[0,0])
            correction=alignment_primitive(state,pose.yaw,map_yaw,
                                            self.cfg['lattice_headings'],
                                            self.robot['max_angular_speed'])
            if correction is not None:
                self.segments.insert(0,(state,state,correction))
                self.log('lattice_heading_alignment',heading_error_rad=
                         correction['samples'][0][2]-correction['samples'][-1][2])
        return super()._activate(pose,goal,now)
    def make_planner(self):
        planner=super().make_planner()
        if STRAIGHT_ONLY:
            from localization_experiment.straight_primitives import straight_library
            planner=self.planner_type().configure(straight_library(planner.library))
        return planner
    def global_plan(self,pose,goal,now):
        if MAP_CORRECTIONS:
            try:
                tf=self.tf.lookup_transform('map','odom_lidar',Time()).transform
                q=tf.rotation;angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                current=(tf.translation.x,tf.translation.y,angle)
            except TransformException:return
            ref=self.map_correction_reference
            if ref is None:self.map_correction_reference=current
            elif math.dist(current[:2],ref[:2])>.1 or abs(math.atan2(math.sin(current[2]-ref[2]),math.cos(current[2]-ref[2])))>math.radians(1):
                # Acquire/validate recovery inputs before changing any state.
                # A delayed TF must leave no half-cleared recovery grid.
                recovery_grid=None
                if self.supervisor.state in ('RECOVERY_BACKUP','RECOVERY_ROTATE'):
                    start=(pose.x,pose.y)
                    recovery_grid=Grid(self.local_map(self.map,start),start,start,navigation_radius(self.sigma,self.robot,self.cfg),self.cfg,hard_radius=physical_radius(self.robot,self.cfg),free_scan=self.scan_free_evidence(),occupied_scan=self.obstacles())
                    recovery_grid.allow_clear_start(start)
                self.map_correction_reference=current;self.map_correction_hold=self.map_time
                # Normal map->odom refinements do not change the fixed map frame.
                # Preserve measured free history; new occupied observations
                # remain authoritative. Large reanchors invalidate history.
                jump=math.dist(current[:2],ref[:2])>self.cfg['pose_position_jump_tolerance'] or abs(math.atan2(math.sin(current[2]-ref[2]),math.cos(current[2]-ref[2])))>math.radians(self.cfg['pose_heading_jump_tolerance_degrees'])
                self._discard_prefetch('map_correction')
                self.active=None;self.active_samples=None;self.active_goal=None;self.terminal_goal=None
                self.segments=[];self.path=[];self.grid=recovery_grid;self.pending_goal=None;self.pending_started=None
                self.macro_subgoal=None;self.macro_goal=None;self.planner=self.make_planner();self.last_plan=-float('inf')
                self.route_reason='map_correction_replan';self.supervisor.blocked_since=None
                self.emit(reason='map_correction_replan');self.publish_path()
                self.log('exploration_map_correction',previous=ref,current=current)
                return
            if self.map_correction_hold is not None:
                if self.map_time<=self.map_correction_hold:
                    self.supervisor.blocked_since=None;self.route_reason='map_correction_wait'
                    if self.supervisor.state in ('RECOVERY_BACKUP','RECOVERY_ROTATE'):return super().global_plan(pose,goal,now)
                    return
                self.map_correction_hold=None
        result=super().global_plan(pose,goal,now)
        if ADAPTIVE_HORIZON:
            executing=self.active is not None or self.terminal_goal is not None
            waiting=self.route_reason.startswith('search') or self.route_reason=='incremental_search_timeout'
            transition=self.horizon_fallback.update(self.get_clock().now().nanoseconds*1e-9,(pose.x,pose.y),waiting,executing)
            if transition:
                self.log('exploration_horizon_'+transition,base_horizon=self.cfg['lattice_horizon_m'],effective_horizon=self.horizon_fallback.limit(self.cfg['lattice_horizon_m']))
                if transition=='reduced':
                    self._discard_prefetch('horizon_fallback');self.pending_goal=None;self.pending_started=None;self.macro_subgoal=None;self.macro_goal=None
                    self.segments=[];self.path=[];self.planner=self.make_planner();self.last_plan=-float('inf');self.publish_path()
        self.short_guidance_connector(pose)
        return result
    def short_guidance_connector(self,pose):
        if self.supervisor.state not in ('GOTO_WAYPOINT','OBSTACLE_AVOID') or self.grid is None or self.terminal_goal is not None or self.macro_subgoal is None:return False
        if self.active is not None and self.active[1]['kind']!='rotate':return False
        start=(pose.x,pose.y);target=tuple(self.macro_subgoal);distance=math.dist(start,target)
        if not .2<distance<=min(self.cfg['lattice_horizon_m'],self.cfg['lattice_goal_tolerance_m']+self.grid.res/2):return False
        if not self.grid.line_clear(start,target):return False
        self.active=None;self.active_samples=None;self.segments=[];self.pending_goal=None;self.pending_started=None
        self.terminal_goal=target;self.path=[start,target];self.route_reason='local_guidance_connector'
        self.publish_path();return True
    def scan_free_evidence(self):
        # Same exact-first / <=0.15 s fallback as baseline obstacle handling.
        scan=self.scan
        try:t=self.tf.lookup_transform('map',scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform
        except TransformException:
            latest=self.tf.lookup_transform('map',scan.header.frame_id,Time())
            ts=latest.header.stamp.sec+latest.header.stamp.nanosec*1e-9
            ss=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            if not 0<=ss-ts<=.15:raise
            t=latest.transform
        q=t.rotation;angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return dict(origin=(t.translation.x,t.translation.y),yaw=angle,angle_min=scan.angle_min,angle_increment=scan.angle_increment,ranges=scan.ranges,range_max=scan.range_max)
    def virtual_goal(self,grid,start,goal):
        m=self.map;q=m.info.origin.orientation
        a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        full=MapGrid(np.asarray(m.data).reshape(m.info.height,m.info.width),m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y,a))
        try:
            scan=self.scan_free_evidence()
            full=supplement_free(full,scan,float(scan['range_max']),self.cfg['live_scan_free_margin'])
            # Conservative 2x aggregation keeps global search bounded. Any
            # occupied fine cell blocks its coarse cell; unknown remains
            # unknown unless all four contributing cells were observed free.
            a=full.data;h,w=a.shape;pad=np.pad(a,((0,h%2),(0,w%2)),constant_values=-1)
            blocks=pad.reshape(pad.shape[0]//2,2,pad.shape[1]//2,2)
            occupied=(blocks>=self.cfg['occupied_threshold']).any(axis=(1,3))
            all_free=((blocks>=0)&(blocks<=25)).all(axis=(1,3))
            coarse=np.where(occupied,100,np.where(all_free,0,-1))
            full=MapGrid(coarse,full.resolution*2,full.origin)
        except Exception as exc:
            self.log('exploration_scan_evidence_unavailable',reason=str(exc));return None
        now=self.get_clock().now().nanoseconds/1e9
        # Guidance and the executing lattice must agree on local obstacle
        # clearance. The global map alone omits fresh local hits and uses a
        # different resolution; previously it could repeatedly select a
        # corridor that the unchanged execution checks necessarily rejected.
        yy,xx=np.indices(full.data.shape)
        centres=full.world(np.column_stack((xx.ravel(),yy.ravel())))
        local=(centres-grid.origin)@grid.rot
        cells=np.floor((local-grid.lower)/grid.res).astype(int)
        inside=((cells>=0).all(axis=1)&(cells[:,0]<grid.blocked.shape[1])&
                (cells[:,1]<grid.blocked.shape[0]))
        executable=np.ones(len(centres),dtype=bool)
        executable[inside]=(grid.nearest_distance(local[inside])>
                            grid.radius+full.resolution/2)
        heading_options=(dict(pose_yaw=self.pose.yaw,nominal_speed=min(self.cfg['lattice_cruise_speed'],self.robot['max_linear_speed']),max_turn_rate=self.robot['max_angular_speed']) if HEADING_GUIDANCE else {})
        route,info=self.guidance.choose(full,start,goal,navigation_radius(self.sigma,self.robot,self.cfg),self.guidance_cfg,now,connector_clear=grid.line_clear,execution_mask=executable.reshape(full.data.shape),**heading_options)
        subgoal=None
        # Farthest visible point no farther than the requested local horizon.
        effective_horizon=self.horizon_fallback.limit(self.cfg['lattice_horizon_m']) if ADAPTIVE_HORIZON else self.cfg['lattice_horizon_m']
        for p in route:
            if math.dist(start,p)>effective_horizon:break
            if grid.free(grid.cell(p)) and grid.line_clear(start,p):subgoal=tuple(p)
        if subgoal is not None and math.dist(start,subgoal)<.35:subgoal=None
        info.update(sim_time=now,start=start,goal=goal,subgoal=subgoal,effective_horizon_m=effective_horizon,known_free_route_length_cells=len(route))
        if self.full_guidance_observer is not None:
            self.full_guidance_decision_id+=1
            pose_yaw=float(self.pose.yaw)
            diagnostic=dict(self.guidance.last_full_state or {})
            diagnostic.update(timestamp=now,decision_id=self.full_guidance_decision_id,
                robot_pose={'x':float(start[0]),'y':float(start[1]),'yaw':pose_yaw},
                mission_target=list(map(float,goal)),navigation_state=self.supervisor.state,
                current_guidance_target=(None if self.guidance.target is None else list(self.guidance.target)),
                goal_connectivity_status=info.get('reason')=='known_free_goal_route',
                online_result=info,selected_stage=(None if subgoal is None else list(subgoal)),
                route_to_selected_or_goal=route)
            overhead_ms,revision=self.full_guidance_observer.submit(full,executable.reshape(full.data.shape),diagnostic)
            self.log('full_guidance_observation',decision_id=self.full_guidance_decision_id,
                     map_revision=revision,enqueue_overhead_ms=overhead_ms)
        self.guidance_audit.write(json.dumps(info)+'\n');self.guidance_audit.flush()
        return subgoal
    def immediate_progress_segments(self,source,start,goal):
        # Direct motion must follow global guidance, never shortcut to the
        # mission point across an unresolved obstacle.
        if self.macro_subgoal is None:return None
        return super().immediate_progress_segments(source,start,self.macro_subgoal)
if __name__=='__main__':raise SystemExit(main(Explorer))
