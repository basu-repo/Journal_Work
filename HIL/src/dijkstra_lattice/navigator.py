"""Dijkstra lattice navigation base using live map, LiDAR and pose."""
import argparse,json,math,time,threading
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import OccupancyGrid,Odometry,Path as PathMsg
from sensor_msgs.msg import LaserScan,NavSatFix
from std_msgs.msg import Bool
from tf2_ros import Buffer,TransformListener,TransformException
from .core import Pose,choose_command,nearest_obstacle_sector,corridor_obstacle,pose_motion_consistent,physical_radius,navigation_radius,wrap
from .grid import Grid
from .supervisor import Supervisor
from .gnss_cycle import GnssCycle, acquire_holds_supervisor
from .waypoint_verification import WaypointVerifier
from .graph_search import GraphSearchBase
from .directional_corridors import DirectionalCorridors


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9


class Navigator(DirectionalCorridors,Node):
    planner_type=GraphSearchBase
    def make_planner(self):return self.planner_type()
    def __init__(self,cfg,robot,output):
        super().__init__('dstar_navigation');self.cfg=cfg;self.robot=robot;self.output=output
        self.localization_group=MutuallyExclusiveCallbackGroup()
        robot['radius']=max(math.hypot(*p) for p in robot['footprint'])+robot['footprint_padding']
        robot['settings']=cfg;self.supervisor=Supervisor(robot)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.scan=None;self.map=None;self.map_time=0.;self.odom_time=0.;self.localization_time=0.;self.localization_ok=False
        self.clock_started=None;self.begin=time.monotonic();self.started=False;self.exit_code=None
        self.sigma=0.;self.ekf_sigma=0.;self.previous=(0.,0.);self.enable_state=None;self.index=0;self.completed=[];self.grid=None;self.path=[];self.route_reason='waiting';self.planner=self.make_planner();self.search_pending_count=0
        self.gnss_mode=cfg.get('gnss_mode')
        self.target_gnss=self.gnss_mode=='target'
        self.gnss=(GnssCycle(cfg['gnss_segments'],start_acquiring=not self.target_gnss)
                   if self.gnss_mode in ('periodic','target') else None)
        self.waypoint_verifier=(WaypointVerifier(cfg) if self.gnss and
                                cfg.get('waypoint_gnss_verification',False) else None)
        self.target_gnss_ready=False;self.target_gnss_exit=None
        self.target_heading_pending=False;self.target_heading_not_before=0.
        self.last_plan=-float('inf');self.last_log=-float('inf');self.target_started=0.;self.progress_pose=None;self.progress_time=0.
        self.progress_best_distance=None
        self.last_state=None;self.pose=None
        self.pose_pair=None;self.pose_pair_time=None;self.pose_hold_until=-float('inf')
        self.transform_failed_since=None;self.global_checks=0;self.route_searches=0;self.route_sequence=0
        self.last_readiness_log=-float('inf');self.last_missing_inputs=None;self.last_gnss_wait_log=-float('inf')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable=self.create_publisher(Bool,'/navigation/enabled',qos);self.command=self.create_publisher(Twist,'/cmd_vel',10)
        self.gnss_gate=self.create_publisher(Bool,'/localization/gnss_request',qos) if self.gnss else None
        if self.target_gnss:self.gnss_gate.publish(Bool(data=False))
        self.path_pub=self.create_publisher(PathMsg,'/dstar/path',qos)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,qos)
        # Availability is based on receipt of the live fused stream.  Its
        # source pose may originate from a delayed 3D scan, while the EKF
        # continuously propagates it with wheel velocity and IMU yaw rate.
        self.create_subscription(Odometry,cfg.get('odom_topic','/odom'),
                                 lambda _m:setattr(self,'odom_time',time.monotonic()),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odometry/global',self.covariance,qos_profile_sensor_data)
        self.create_subscription(Bool,'/localization/ready',self.health,qos,
                                 callback_group=self.localization_group)
        if self.gnss:self.create_subscription(NavSatFix,'/localization/fix',self.gnss_fix,
                                              qos_profile_sensor_data,
                                              callback_group=self.localization_group)
        # A simulation-clock timer can remain permanently ready when headless
        # Gazebo runs faster than real time, starving queued sensor and
        # localization callbacks in this single-thread executor.
        self.control_period=1/cfg['controller_frequency']
        self.control_stop=threading.Event();self.control_thread=None
        self.log('waiting_for_readiness',delay_seconds=cfg['startup_delay'],
                 obstacle_body_clearance_m=cfg.get('obstacle_body_clearance'),
                 minimum_centre_clearance_m=physical_radius(robot,cfg))
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def start_control(self):
        if self.control_thread is None:
            self.control_thread=threading.Thread(target=self._control_loop,
                                                 name='navigation_control',daemon=True)
            self.control_thread.start()
    def stop_control(self):
        self.control_stop.set()
        if self.control_thread is not None:self.control_thread.join(timeout=2.)
    def _control_loop(self):
        deadline=time.monotonic()
        while rclpy.ok() and self.exit_code is None and not self.control_stop.is_set():
            try:self.tick()
            except Exception as exc:
                self.log('controller_exception',error=repr(exc));self.exit_code=4;break
            deadline+=self.control_period
            delay=deadline-time.monotonic()
            if delay<=0:deadline=time.monotonic()
            self.control_stop.wait(max(0.,min(self.control_period,delay)))
    def health(self,msg):self.localization_ok=msg.data;self.localization_time=time.monotonic()
    def gnss_fix(self,msg):self.gnss.note_fix()
    def publish_gnss_gate(self):
        if self.gnss:self.gnss_gate.publish(Bool(data=self.gnss.state=='ACQUIRE'))
    def defer_periodic_gnss(self):return False
    def enter_gnss_acquire(self,now,reason):
        if self.gnss and self.gnss.acquire(now,reason):
            self.target_gnss_ready=False
            self.publish_gnss_gate();self.log('gnss_transition',state='ACQUIRE',reason=reason,segment_id=self.gnss.segment_id)
    def invalidate_after_reanchor(self):
        self.path=[];self.grid=None;self.planner=self.make_planner()
        self.last_plan=-float('inf');self.route_reason='gnss_reanchored'
        self.publish_path()
    def advance_waypoint(self,now,local_pose):
        completed_goal=self.target()
        self.index+=1;self.path=[];self.emit()
        if self.index==len(self.cfg['targets']):
            if self.target_gnss:
                self.gnss.coast(now,(local_pose.x,local_pose.y),self.route_sequence,'mission_complete')
                self.publish_gnss_gate()
            self.finish(0,'estimated_mission_completed');return
        goal=self.target();self.target_started=now
        self.progress_time=now;self.progress_pose=local_pose;self.progress_best_distance=None
        if self.target_gnss:
            self.target_gnss_exit=completed_goal
        else:self.enter_gnss_acquire(now,'waypoint_changed')
        self.supervisor.waypoint_advanced(now,goal)
        self.log('goal_sent',target=self.cfg['targets'][self.index]['name'])
    def covariance(self,msg):
        # In GNSS localization this is also the planner's fused odometry
        # stream, so one callback supplies both covariance and liveness.
        if self.cfg.get('odom_topic')=='/odometry/global':
            self.odom_time=time.monotonic()
        c=msg.pose.covariance
        if all(math.isfinite(v) for v in [c[0],c[1],c[6],c[7]]):
            self.ekf_sigma=math.sqrt(max(0.,float(np.linalg.eigvalsh([[c[0],c[1]],[c[6],c[7]]]).max())))
            self.sigma=self.ekf_sigma
    def on_scan(self,msg):
        if len(msg.ranges) and any((math.isfinite(x) and msg.range_min<=x<=msg.range_max) or x==float('inf') for x in msg.ranges):self.scan=msg
    def on_map(self,msg):
        if msg.header.frame_id==self.robot.get('map_frame','map') and msg.info.resolution>0 and len(msg.data)>0 and len(msg.data)==msg.info.width*msg.info.height:
            self.map=msg;self.map_time=self.now()
    def log(self,event,**data):
        record=dict(event=event,wall_time=time.time(),sim_time=self.now(),**data)
        with (self.output/'events.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
        print(json.dumps(record),flush=True)
    def emit(self,v=0.,w=0.,enabled=False):
        if enabled!=self.enable_state:
            self.enable.publish(Bool(data=enabled));self.enable_state=enabled
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.command.publish(msg);self.previous=(float(v),float(w))
    def transform_pose(self,frame):
        t=self.tf.lookup_transform(frame,self.robot['base_frame'],Time()).transform
        return Pose(t.translation.x,t.translation.y,yaw(t.rotation))
    def obstacles(self):
        scan=self.scan;t=self.tf.lookup_transform(self.robot.get('map_frame','map'),scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform
        angle=yaw(t.rotation);r=np.asarray(scan.ranges);valid=np.isfinite(r)&(r>=scan.range_min)&(r<=scan.range_max)
        angles=scan.angle_min+np.arange(len(r))*scan.angle_increment+angle
        return np.column_stack((t.translation.x+r[valid]*np.cos(angles[valid]),t.translation.y+r[valid]*np.sin(angles[valid])))
    def scan_free_evidence(self):
        scan=self.scan
        t=self.tf.lookup_transform(self.robot.get('map_frame','map'),scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform
        return dict(origin=(t.translation.x,t.translation.y),yaw=yaw(t.rotation),
                    angle_min=scan.angle_min,angle_increment=scan.angle_increment,
                    ranges=scan.ranges,range_max=scan.range_max)
    def missing_inputs(self):
        now=self.now();missing=[]
        # In periodic mode the accepted GNSS fix seeds/corrects the estimator,
        # then the receiver is intentionally absent while COAST executes the
        # committed route.  A fresh GNSS heartbeat is required only when GNSS
        # is configured as a continuous localization source.
        if (self.cfg['localization_mode']=='gnss' and self.gnss is None and
                not (self.localization_ok and
                     time.monotonic()-self.localization_time<self.cfg['sensor_timeout'])):
            missing.append('gnss_localization')
        if self.clock_started is None:missing.append('clock')
        if self.scan is None or not 0<=now-stamp(self.scan)<self.cfg['sensor_timeout']:missing.append('scan')
        localization_fresh=(self.localization_ok and
                            time.monotonic()-self.localization_time<self.cfg['sensor_timeout'])
        if self.cfg['localization_mode']=='gnss':
            # localization_input is the single health authority: it already
            # requires fresh wheel, IMU and fused global odometry plus bounded
            # covariance (and GNSS when the target gate is active).
            if not localization_fresh:missing.append('odom')
        elif time.monotonic()-self.odom_time>=self.cfg['sensor_timeout']:
            missing.append('odom')
        if self.map is None or not 0<=now-self.map_time<self.cfg['map_timeout']:missing.append('slam_map')
        return missing
    def ready(self):return not self.missing_inputs()
    def finish(self,code,event):
        self.emit();self.exit_code=code;self.log(event)
    def target(self):
        t=self.cfg['targets'][self.index];return (t['x'],t['y'])
    def publish_path(self):
        msg=PathMsg();msg.header.frame_id=self.robot.get('map_frame','map');msg.header.stamp=self.get_clock().now().to_msg()
        for x,y in self.path:
            p=PoseStamped();p.header=msg.header;p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.;msg.poses.append(p)
        self.path_pub.publish(msg)
    def global_plan(self,pose,goal,now):
        if now-self.last_plan<self.cfg['replan_interval'] and self.path:return
        self.last_plan=now;self.global_checks+=1
        start=(pose.x,pose.y)
        try:free_scan=self.scan_free_evidence()
        except TransformException:free_scan=None
        try:occupied_scan=self.obstacles()
        except TransformException:occupied_scan=None
        try:
            grid=Grid(self.map,start,goal,navigation_radius(self.sigma,self.robot,self.cfg),self.cfg,
                      hard_radius=physical_radius(self.robot,self.cfg),
                      free_scan=free_scan,occupied_scan=occupied_scan)
        except ValueError as exc:
            self.grid=None;self.path=[];self.route_reason=str(exc);self.publish_path();return
        self.grid=grid
        source,target=grid.cell(start),grid.cell(goal)
        grid.allow_clear_start(start)
        update=self.planner.update(grid,source,target,start,goal)
        # Retain a route only while every remaining swept segment is still
        # clear. A changed map or deviation starts a fresh Dijkstra search.
        if self.path:
            position=np.array(start);candidates=[]
            for i,(a,b) in enumerate(zip(self.path,self.path[1:])):
                a,b=np.array(a),np.array(b);edge=b-a
                t=float(np.clip((position-a)@edge/max(float(edge@edge),1e-9),0.,1.))
                distance=float(np.linalg.norm(position-(a+t*edge)))
                if distance<=self.cfg['route_deviation_limit'] and grid.line_clear(position,b):
                    candidates.append((distance,i))
            if candidates:
                _,i=min(candidates)
                remaining=[start]+self.path[i+1:]
                if all(grid.line_clear(a,b) for a,b in zip(remaining,remaining[1:])):
                    self.path=remaining;self.route_reason='dstar_route_retained'
                    self.publish_path();return
        self.route_searches+=1
        if not grid.free(target):
            self.path=[];self.route_reason='goal_occupied'
        elif not grid.free(source):
            self.path=[];self.route_reason='start_occupied'
        elif not self.planner.compute(self.cfg['planning_max_expansions'],self.cfg['planning_time_limit']):
            self.path=[];self.route_reason='incremental_search_timeout'
            self.search_pending_count+=1
            self.log('dstar_search_pending',update=update,expansions=self.planner.expansions)
        else:
            self.path,self.route_reason=self.planner.path(start,goal)
        self.route_sequence+=1;self.publish_path()
    def motion_clear(self,xy,radius=None):
        if radius is None:radius=navigation_radius(self.sigma,self.robot,self.cfg)
        return self.grid is not None and self.grid.motion_clear(xy,radius)
    def local_target(self,pose,lookahead=None):
        path=self.path;grid=self.grid
        if not path:return None
        start=np.array([pose.x,pose.y]);last=start;left=self.cfg['lookahead'] if lookahead is None else lookahead
        # Skip path vertices only when the same inflated grid validates the shortcut.
        visible=[i for i,p in enumerate(path) if grid.line_clear(start,p)]
        # If retained SLAM inflation overlaps the current pose, follow the next
        # ordered route vertex; the fresh LiDAR rollout still rejects collisions.
        i=max(visible) if visible else 0
        for p in path[i:]:
            p=np.array(p);distance=float(np.linalg.norm(p-last))
            if distance>=left:return tuple(last+(p-last)*left/max(distance,1e-9))
            left-=distance;last=p
        return tuple(last)
    def trajectory_clear(self,states):
        if self.grid is None:return False
        if np.max(np.linalg.norm(states-states[0],axis=1))<1e-6:
            return self.grid.motion_clear(states,self.grid.hard_radius)
        return self.grid.motion_clear(states)
    def propose_command(self,pose,target,points):
        return choose_command(pose,target,points,self.previous,self.robot,self.cfg,
                              trajectory_clear=self.trajectory_clear) if target else (0.,0.,'blocked')
    def note_goal_progress(self,distance,now,local_pose):
        """Count progress only when the robot gets closer to this waypoint."""
        if (self.progress_best_distance is None or
                distance<=self.progress_best_distance-self.cfg['progress_distance']):
            self.progress_best_distance=distance
            self.progress_time=now;self.progress_pose=local_pose
            return True
        return False
    def reset_recovery_cycle(self,now,pose,local_pose):
        """Optional model-specific restart after a bounded recovery cycle."""
        return False
    def tick(self):
        if self.exit_code is not None:return
        now=self.now();wall=time.monotonic()
        if self.clock_started is None and now>0.:
            self.clock_started=wall;self.log('world_clock_started')
        if (self.waypoint_verifier and self.waypoint_verifier.pending and
                now-self.waypoint_verifier.pending['started_at']>=
                self.cfg['waypoint_gnss_timeout_s']):
            self.finish(3,'waypoint_gnss_timeout');return
        missing=self.missing_inputs()
        if missing:
            if missing!=self.last_missing_inputs or wall-self.last_readiness_log>=5.0:
                self.log('waiting_for_inputs',missing=missing)
                self.last_missing_inputs=missing;self.last_readiness_log=wall
            self.emit()
            if not self.started and wall-self.begin>self.cfg['readiness_timeout']:self.finish(2,'readiness_timeout')
            elif (self.started and now-self.target_started>self.cfg['waypoint_timeout'] and
                  not (self.waypoint_verifier and self.waypoint_verifier.pending)):
                self.finish(3,'goal_timeout')
            return
        try:pose=self.transform_pose(self.robot.get('map_frame','map'));local_pose=self.transform_pose(self.cfg.get('odom_frame','odom'));points=self.obstacles()
        except TransformException:
            if self.transform_failed_since is None:self.transform_failed_since=now
            self.emit();return
        self.transform_failed_since=None
        if self.pose_pair is None:
            self.pose_hold_until=now+self.cfg['pose_stable_seconds']
            self.log('pose_health_hold',reason='initializing',hold_until=self.pose_hold_until)
        elif now>self.pose_pair_time:
            healthy,reason=pose_motion_consistent(
                self.pose_pair,(pose,local_pose),now-self.pose_pair_time,
                self.robot['max_linear_speed'],self.robot['max_angular_speed'],
                self.cfg['pose_position_jump_tolerance'],math.radians(self.cfg['pose_heading_jump_tolerance_degrees']))
            if not healthy:
                self.pose_hold_until=now+self.cfg['pose_stable_seconds']
                self.path=[];self.planner=self.make_planner();self.publish_path()
                self.log('pose_health_hold',reason=reason,hold_until=self.pose_hold_until,
                         map_pose=[pose.x,pose.y,pose.yaw],odom_pose=[local_pose.x,local_pose.y,local_pose.yaw])
        if self.pose_pair_time is None or now>self.pose_pair_time:
            self.pose_pair=(pose,local_pose);self.pose_pair_time=now
        if now<self.pose_hold_until:
            self.emit();return
        self.pose=pose
        self.sigma=self.ekf_sigma
        if self.gnss and self.gnss.state=='COAST' and not self.target_gnss:
            cfg=self.cfg['gnss_segments']
            self.sigma=max(self.sigma,cfg['drift_sigma_per_s']*max(0.,now-self.gnss.coast_started_at)
                           +cfg['drift_sigma_per_m']*self.gnss.distance((local_pose.x,local_pose.y)))
        if not self.started:
            if wall-self.clock_started<self.cfg['startup_delay']:self.emit();return
            self.started=True;self.target_started=now;self.progress_pose=local_pose;self.progress_time=now
            self.progress_best_distance=None
            self.log('ready');self.log('goal_sent',target=self.cfg['targets'][self.index]['name'])
        if (now-self.target_started>self.cfg['waypoint_timeout'] and
                not (self.waypoint_verifier and self.waypoint_verifier.pending)):
            self.finish(3,'goal_timeout');return
        goal=self.target();distance=math.dist((pose.x,pose.y),goal)
        if self.target_gnss:
            radius=float(self.cfg['target_gnss_radius_m'])
            # A drifted denied estimate can falsely claim target-zone entry.
            # Once fresh GNSS corrects the pose outside the zone, close the
            # receiver immediately, retain the EKF correction, and continue
            # denied navigation until the corrected pose re-enters 2 m.
            if (self.target_gnss_ready and self.gnss.state=='ACQUIRE' and
                    distance>radius):
                self.gnss.coast(now,(local_pose.x,local_pose.y),self.route_sequence,
                                'corrected_outside_target_zone')
                self.publish_gnss_gate();self.target_gnss_ready=False
                # Preserve the corrected EKF pose.  The global /reset service
                # belongs to robot_localization in this launch and would erase
                # the GNSS correction rather than clear only SLAM geometry.
                self.invalidate_after_reanchor()
                self.log('navigation_reanchored',reason='gnss_corrected_outside_target_zone')
                self.log('target_gnss_disabled',reason='corrected_outside_target_zone',
                         corrected_distance=distance,radius_m=radius)
            if (self.target_gnss_exit is not None and
                    math.dist((pose.x,pose.y),self.target_gnss_exit)>radius):
                self.gnss.coast(now,(local_pose.x,local_pose.y),self.route_sequence)
                self.publish_gnss_gate();self.target_gnss_exit=None
                self.target_gnss_ready=False
                self.log('target_gnss_disabled',reason='left_target_zone',radius_m=radius)
            if self.target_gnss_exit is None and distance<=radius and self.gnss.state=='COAST':
                self.enter_gnss_acquire(now,'target_zone_entry')
                self.log('target_gnss_enabled',target=self.cfg['targets'][self.index]['name'],
                         estimated_distance=distance,radius_m=radius)
            if self.gnss.state=='ACQUIRE' and not self.target_gnss_ready:
                ready=(self.gnss.fix_count>=self.cfg['gnss_segments']['min_fixes'] and
                       self.gnss.started_at is not None and
                       now-self.gnss.started_at>=self.cfg['gnss_segments']['acquire_s'] and
                       self.localization_ok and math.isfinite(self.ekf_sigma) and
                       self.ekf_sigma<self.cfg['gnss_segments']['sigma_limit'])
                if not ready:
                    self.emit();return
                self.target_gnss_ready=True;self.path=[];self.last_plan=-float('inf')
                self.target_heading_pending=True
                self.target_heading_not_before=now+self.cfg['pose_stable_seconds']
                self.log('target_gnss_locked',target=self.cfg['targets'][self.index]['name'],
                         fixes=self.gnss.fix_count,position_sigma=self.ekf_sigma)
                self.emit();return
        # GNSS fixes position, then the robot physically turns toward the
        # target using the corrected position and its fused yaw.  Never write
        # an invented yaw into the estimator.
        if self.target_heading_pending:
            if now<self.target_heading_not_before:
                self.emit();return
            distance=math.dist((pose.x,pose.y),goal)
            if distance>self.cfg['goal_tolerance']-self.cfg['arrival_reserve']:
                desired=math.atan2(goal[1]-pose.y,goal[0]-pose.x)
                error=wrap(desired-pose.yaw)
                tolerance=math.radians(self.cfg['waypoint_heading_degrees'])
                if abs(error)>tolerance:
                    speed=min(self.robot['max_angular_speed'],
                              max(.15,float(self.cfg['recovery_turn_speed'])))
                    self.emit(0.,math.copysign(speed,error),True)
                    return
            self.target_heading_pending=False
            self.path=[];self.last_plan=-float('inf')
            self.log('target_heading_aligned',target=self.cfg['targets'][self.index]['name'],
                     corrected_distance=distance)
        if self.waypoint_verifier and self.waypoint_verifier.pending:
            name=self.cfg['targets'][self.index]['name']
            pending=self.waypoint_verifier.pending
            outcome,corrected_distance=self.waypoint_verifier.check(
                now,goal,(pose.x,pose.y),self.gnss.fix_count,
                self.gnss.state=='ACQUIRE',self.localization_ok,self.ekf_sigma)
            if outcome=='waiting':self.emit();return
            if outcome=='timeout':self.finish(3,'waypoint_gnss_timeout');return
            self.invalidate_after_reanchor()
            if outcome=='accepted':
                self.log('waypoint_gnss_accepted',target=name,
                         corrected_distance=corrected_distance,
                         precheck_distance=pending['estimated_distance'],
                         accepted_fixes=self.gnss.fix_count)
                self.completed.append(name)
                self.log('waypoint_reached',target=name,
                         estimated_distance=corrected_distance,
                         estimated_xy=[pose.x,pose.y],arrival_basis='fresh GNSS-fused localization estimate')
                self.advance_waypoint(now,local_pose)
            else:
                self.log('waypoint_gnss_rejected',target=name,
                         corrected_distance=corrected_distance,
                         precheck_distance=pending['estimated_distance'],
                         correction_m=math.dist(pending['estimated_xy'],(pose.x,pose.y)),
                         accepted_fixes=self.gnss.fix_count)
                self.progress_time=now;self.progress_pose=local_pose
                self.progress_best_distance=corrected_distance
                self.supervisor.waypoint_advanced(now,goal)
                self.emit()
            return
        if distance<=self.cfg['goal_tolerance']-self.cfg['arrival_reserve']:
            name=self.cfg['targets'][self.index]['name']
            if self.waypoint_verifier:
                self.waypoint_verifier.begin(now,goal,(pose.x,pose.y),distance)
                self.log('waypoint_gnss_check_started',target=name,
                         precheck_distance=distance,estimated_xy=[pose.x,pose.y])
                self.enter_gnss_acquire(now,'waypoint_verification')
                self.path=[];self.publish_path();self.emit();return
            self.completed.append(name)
            self.log('waypoint_reached',target=name,estimated_distance=distance,estimated_xy=[pose.x,pose.y],arrival_basis='localization estimate')
            self.advance_waypoint(now,local_pose);return
        # GNSS acquisition must run before global planning.  Otherwise a long
        # lattice repair can monopolize the single executor exactly when fresh
        # fixes need to be consumed, making a healthy localization stream look
        # like an input freeze.
        if (self.gnss and not self.target_gnss and self.gnss.state=='ACQUIRE' and
                (self.path or self.gnss.segment_id>0)):
            event=self.gnss.step(now,(local_pose.x,local_pose.y),self.route_sequence,
                                 bool(self.path),self.sigma)
            if event:
                self.publish_gnss_gate()
                self.log('gnss_transition',state=self.gnss.state,reason=event,
                         segment_id=self.gnss.segment_id,accepted_fixes=self.gnss.fix_count,
                         coast_distance_m=self.gnss.distance((local_pose.x,local_pose.y)))
            if self.gnss.state=='ACQUIRE' and acquire_holds_supervisor(self.supervisor.state):
                if wall-self.last_gnss_wait_log>=1.:
                    self.last_gnss_wait_log=wall
                    self.log('gnss_acquiring',accepted_fixes=self.gnss.fix_count,
                             position_sigma=self.sigma,has_path=bool(self.path))
                self.emit();return
        self.global_plan(pose,goal,now)
        self.log_directions(pose,points)
        if self.gnss and not self.target_gnss and not (self.gnss.state=='COAST' and self.defer_periodic_gnss()):
            event=self.gnss.step(now,(local_pose.x,local_pose.y),self.route_sequence,
                                 bool(self.path),self.sigma)
            if event:
                self.publish_gnss_gate()
                self.log('gnss_transition',state=self.gnss.state,reason=event,
                         segment_id=self.gnss.segment_id,accepted_fixes=self.gnss.fix_count,
                         coast_distance_m=(self.gnss.last_coast_distance if self.gnss.state=='ACQUIRE'
                                           else self.gnss.distance((local_pose.x,local_pose.y))))
            if self.gnss.state=='ACQUIRE' and acquire_holds_supervisor(self.supervisor.state):
                self.emit();return
        if self.route_reason=='incremental_search_timeout':
            self.emit();return
        target=self.local_target(pose)
        proposed=self.propose_command(pose,target,points)
        self.note_goal_progress(distance,now,local_pose)
        if now-self.progress_time>self.cfg['progress_timeout']:proposed=(0.,0.,'stalled_stop')
        sector,nearest=nearest_obstacle_sector(pose,points)
        corridor_sector,corridor_distance=corridor_obstacle(
            pose,points,physical_radius(self.robot,self.cfg),self.cfg['obstacle_avoid_distance'])
        # A periodic GNSS correction is deliberately deferred while a checked
        # lattice route remains committed.  Treat that interval as anchored
        # for supervision so covariance growth cannot discard the route and
        # its prefetched successor midway through execution.  Live corridor
        # collision checks and pose-consistency holds remain independent.
        committed_coast=bool(self.gnss and self.gnss.state=='COAST' and
                             self.defer_periodic_gnss())
        v,w,reset=self.supervisor.step(now,pose,local_pose,
                                      ('anchored' if committed_coast or not self.gnss
                                       else 'coasting'),
                                      self.sigma if self.gnss else 0.,points,corridor_distance,proposed[:2],proposed[2],
                                      obstacle_sector=corridor_sector,next_target=goal,latest_lidar_stamp=stamp(self.scan),
                                      trajectory_clear=self.trajectory_clear)
        if self.reset_recovery_cycle(now,pose,local_pose):
            # The model rebuilt its navigation state from the current local
            # pose. Hold one control tick so no command from the retired cycle
            # can leak into the fresh one.
            v,w,reset=0.,0.,True
        if reset:
            self.path=[]
            self.progress_time=now;self.progress_pose=local_pose
            self.progress_best_distance=distance
        if self.supervisor.state!=self.last_state:
            if self.supervisor.state in ('RECOVERY_BACKUP','RECOVERY_ROTATE'):self.path=[]
            self.last_state=self.supervisor.state;self.log('state',state=self.last_state,recoveries=self.supervisor.recovery_count)
        if self.supervisor.state=='RECOVERY_FAILED':self.finish(3,'recovery_failed');return
        # If planning took too long, the command must wait for a new scan.
        if self.now()-stamp(self.scan)>=self.cfg['sensor_timeout']:self.emit();return
        self.emit(v,w,True)
        if now-self.last_log>=1.:
            self.last_log=now
            remaining=(math.dist((pose.x,pose.y),self.path[1])+sum(math.dist(a,b) for a,b in zip(self.path[1:],self.path[2:]))) if len(self.path)>1 else distance
            self.log('progress',target=self.cfg['targets'][self.index]['name'],distance_remaining=distance,planned_path_length=remaining,route_reason=self.route_reason,recoveries=self.supervisor.recovery_count)
            with (self.output/'status.jsonl').open('a') as f:f.write(json.dumps(dict(sim_time=now,pose=[pose.x,pose.y,pose.yaw],target=goal,local_target=target,path=self.path,route_reason=self.route_reason,state=self.supervisor.state,nearest_obstacle=nearest,obstacle_sector=sector,corridor_obstacle_distance=corridor_distance,corridor_obstacle_sector=corridor_sector,map_cells_overridden=self.grid.live_cleared if self.grid else 0,live_obstacle_cells=self.grid.live_obstacle_cells if self.grid else 0,source_cell_relaxed=self.grid.source_cell_relaxed if self.grid else False,planning_clearance=float(self.grid.nearest_distance(self.grid.local((pose.x,pose.y)))) if self.grid else None,source_cell_clearance=float(self.grid.distance[self.grid.cell((pose.x,pose.y))[1],self.grid.cell((pose.x,pose.y))[0]]) if self.grid else None,position_sigma=self.sigma,gnss_state=self.gnss.state if self.gnss else 'continuous',
                gnss_segment_id=self.gnss.segment_id if self.gnss else None,
                command=[v,w]))+'\n')
    def summary(self):
        (self.output/'summary.json').write_text(json.dumps(dict(route_success=False,estimated_route_success=self.exit_code==0,
             planning=dict(source='slam_occupancy_map',algorithm='dstar_original',
                           global_checks=self.global_checks,route_searches=self.route_searches,
                           vertex_expansions=self.planner.expansions,map_repairs=self.planner.repairs,
                           astar_fallbacks=0,search_pending=self.search_pending_count),
             completed_targets=self.completed,requested_targets=[t['name'] for t in self.cfg['targets']],
             gnss_mode=self.cfg.get('gnss_mode','continuous'),gnss_segments=self.gnss.segment_id if self.gnss else 0,
             arrival_basis='localization estimate; awaiting independent simulation verification'),indent=2))


def main(navigator_type=Navigator):
    p=argparse.ArgumentParser();p.add_argument('--mission',type=Path,required=True);p.add_argument('--robot',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args,ros=p.parse_known_args();rclpy.init(args=ros);node=navigator_type(yaml.safe_load(args.mission.read_text()),yaml.safe_load(args.robot.read_text()),args.output)
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node);node.start_control()
    try:
        while rclpy.ok() and node.exit_code is None:executor.spin_once(timeout_sec=.1)
        return node.exit_code if node.exit_code is not None else 130
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):return 130
    finally:
        node.stop_control()
        executor.shutdown()
        node.summary()
        if rclpy.ok():node.emit();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
