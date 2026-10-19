"""Shared state-lattice navigation adapter."""
import json
import math
import copy
import numpy as np
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile,DurabilityPolicy
from tf2_ros import TransformException
from .navigator import Navigator,main
from .grid import Grid
from .core import navigation_radius,physical_radius

class LatticeNavigator(Navigator):
    planner_type=None
    algorithm='state_lattice'
    route_topic='/dijkstra_lattice/path'
    def __init__(self,cfg,robot,output):
        super().__init__(cfg,robot,output)
        self.states=[];self.turn_heading=None;self.pending_goal=None;self.crop_center=None;self.macro_subgoal=None;self.macro_goal=None
        self.destroy_publisher(self.path_pub)
        self.path_pub=self.create_publisher(PathMsg,self.route_topic,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def publish_path(self):
        msg=PathMsg();msg.header.frame_id=self.robot.get('map_frame','map');msg.header.stamp=self.get_clock().now().to_msg()
        for x,y in self.path:
            p=PoseStamped();p.header=msg.header;p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.
            msg.poses.append(p)
        self.path_pub.publish(msg)
    def local_map(self,msg,start):
        # Crop the live SLAM grid around the robot. No saved world data enters
        # the planner; the crop only bounds the lattice search.
        half=self.cfg.get('lattice_window_half_m',10.)
        res=msg.info.resolution
        origin=np.array([msg.info.origin.position.x,msg.info.origin.position.y])
        q=msg.info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        rot=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        if self.crop_center is None or math.dist(start,self.crop_center)>half*.5:
            self.crop_center=tuple(start)
        local=rot.T@(np.asarray(self.crop_center)-origin)
        center=np.floor(local/res).astype(int)
        width,height=msg.info.width,msg.info.height
        n=max(4,int(math.ceil(half/res)))
        x0=max(0,int(center[0]-n));x1=min(width,int(center[0]+n+1))
        y0=max(0,int(center[1]-n));y1=min(height,int(center[1]+n+1))
        if x1<=x0 or y1<=y0: return msg
        cropped=copy.deepcopy(msg)
        data=np.asarray(msg.data,dtype=np.int8).reshape(height,width)
        cropped.data=data[y0:y1,x0:x1].ravel().tolist()
        cropped.info.width=x1-x0;cropped.info.height=y1-y0
        shifted=origin+rot@np.array([x0*res,y0*res])
        cropped.info.origin.position.x=float(shifted[0])
        cropped.info.origin.position.y=float(shifted[1])
        return cropped
    def virtual_goal(self,grid,start,goal):
        distance=math.dist(start,goal)
        horizon=self.cfg.get('lattice_horizon_m',5.)
        if distance<=horizon and grid.free(grid.cell(goal)): return goal
        bearing=math.atan2(goal[1]-start[1],goal[0]-start[0])
        candidates=[]
        for offset in np.linspace(-math.pi/2,math.pi/2,25):
            for radius in (horizon,horizon*.8,horizon*.6):
                point=(start[0]+radius*math.cos(bearing+offset),
                       start[1]+radius*math.sin(bearing+offset))
                cell=grid.cell(point)
                if not grid.free(cell): continue
                center=grid.xy(cell)
                if not grid.line_clear(start,center) and radius<horizon*.8: continue
                candidates.append((abs(offset)*2+math.dist(center,goal)/max(distance,1.),center))
        return min(candidates,key=lambda x:x[0])[1] if candidates else None
    def global_plan(self,pose,goal,now):
        if self.route_reason=='incremental_search_timeout' and self.pending_goal==tuple(goal) and self.planner.grid is not None:
            self.last_plan=now
            if not self.planner.compute(self.cfg['planning_max_expansions'],self.cfg['planning_time_limit']):
                self.search_pending_count+=1
                return
            self.grid=self.planner.grid
            self.states,self.route_reason=self.planner.path()
            self.path=[(pose.x,pose.y)]+[self.grid.xy(st[:2]) for st in self.states[1:]] if self.states else []
            self.pending_goal=None;self.route_sequence+=1;self.publish_path()
            return
        if now-self.last_plan<self.cfg['replan_interval'] and self.path:return
        self.last_plan=now;self.global_checks+=1
        start=(pose.x,pose.y)
        try:free_scan=self.scan_free_evidence()
        except TransformException:free_scan=None
        try:occupied_scan=self.obstacles()
        except TransformException:occupied_scan=None
        distance=math.dist(start,goal)
        scale=min(1.,self.cfg.get('lattice_horizon_m',5.)/max(distance,1e-6))
        seedgoal=(start[0]+(goal[0]-start[0])*scale,
                  start[1]+(goal[1]-start[1])*scale)
        try:
            grid=Grid(self.local_map(self.map,start),start,seedgoal,navigation_radius(self.sigma,self.robot,self.cfg),self.cfg,
                      hard_radius=physical_radius(self.robot,self.cfg),free_scan=free_scan,occupied_scan=occupied_scan)
        except ValueError as exc:
            self.grid=None;self.path=[];self.states=[];self.route_reason=str(exc);self.publish_path();return
        self.grid=grid;grid.allow_clear_start(start)
        map_yaw=math.atan2(grid.rot[1,0],grid.rot[0,0])
        source=(*grid.cell(start),round((pose.yaw-map_yaw)/(math.pi/4))%8)
        if self.macro_goal!=tuple(goal):
            self.macro_goal=tuple(goal);self.macro_subgoal=None
        if (self.macro_subgoal is not None and math.dist(start,self.macro_subgoal)>1.0
                and grid.free(grid.cell(self.macro_subgoal))):
            subgoal=self.macro_subgoal
        else:
            subgoal=self.virtual_goal(grid,start,goal)
            self.macro_subgoal=subgoal
        if subgoal is None:
            self.states=[];self.path=[];self.route_reason='no_local_subgoal';self.publish_path();return
        target=grid.cell(subgoal)
        update=self.planner.update(grid,source,target,start,subgoal)
        self.route_searches+=1
        if not grid.free(target):self.states=[];self.route_reason='goal_occupied'
        elif not grid.free(source[:2]):self.states=[];self.route_reason='start_occupied'
        elif not self.planner.compute(self.cfg['planning_max_expansions'],self.cfg['planning_time_limit']):
            self.states=[];self.route_reason='incremental_search_timeout';self.search_pending_count+=1
            self.pending_goal=tuple(goal)
            self.log('lattice_search_pending',update=update,expansions=self.planner.expansions)
        else:
            self.states,self.route_reason=self.planner.path();self.pending_goal=None
        self.path=[start]+[grid.xy(st[:2]) for st in self.states[1:]] if self.states else []
        self.route_sequence+=1;self.publish_path()
    def local_target(self,pose,lookahead=None):
        self.turn_heading=None
        if len(self.states)<2 or self.grid is None:return None
        current,following=self.states[:2]
        if current[:2]==following[:2]:
            map_yaw=math.atan2(self.grid.rot[1,0],self.grid.rot[0,0])
            self.turn_heading=following[2]*math.pi/4+map_yaw
            return (pose.x+math.cos(self.turn_heading),pose.y+math.sin(self.turn_heading))
        # Track consecutive forward primitives only; do not shortcut a turn.
        i=1
        while i+1<len(self.states) and self.states[i+1][2]==following[2] and self.states[i+1][:2]!=self.states[i][:2] and math.dist((pose.x,pose.y),self.grid.xy(self.states[i+1][:2]))<self.cfg['lookahead']:
            i+=1
        return self.grid.xy(self.states[i][:2])
    def propose_command(self,pose,target,points):
        if self.turn_heading is None:return super().propose_command(pose,target,points)
        error=(self.turn_heading-pose.yaw+math.pi)%(2*math.pi)-math.pi
        if abs(error)<math.radians(6):return (0.,0.,'tracking')
        w=max(-self.robot['max_angular_speed'],min(self.robot['max_angular_speed'],1.2*error))
        w=math.copysign(max(0.15,abs(w)),w)
        if not self.trajectory_clear(np.asarray([[pose.x,pose.y],[pose.x,pose.y]])):return (0.,0.,'blocked')
        return (0.,w,'tracking')
    def summary(self):
        super().summary();path=self.output/'summary.json';data=json.loads(path.read_text())
        data['planning']['algorithm']=self.algorithm
        data['planning']['lattice_primitives']=['forward_cell','rotate_45_degrees']
        path.write_text(json.dumps(data,indent=2))
