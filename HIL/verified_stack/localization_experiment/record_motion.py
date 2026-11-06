"""Read-only sensor diagnostics. No publishers and no estimator feedback."""
import json, sys, os
import numpy as np
from sensor_msgs_py import point_cloud2
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu,PointCloud2
class Recorder(Node):
    def __init__(self,path):
        super().__init__('u_motion_diagnostic')
        self.out=Path(path).open('w')
        self.truth_out=(Path(path).parent/'truth_dense.jsonl').open('w')
        self.create_subscription(PoseStamped,'/simulation/ground_truth/pose',self.truth,qos_profile_sensor_data)
        self.cloud_count=0;self.cloud_dir=Path(path).parent/'raw_clouds'
        if os.environ.get('U_CAPTURE_CLOUD')=='1':
            self.cloud_dir.mkdir(exist_ok=True)
            self.create_subscription(PointCloud2,'/lidar/points',self.cloud,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.wheel,qos_profile_sensor_data)
        self.create_subscription(Imu,'/imu/data',self.imu,qos_profile_sensor_data)
    def truth(self,m):
        # Evaluation-only sink. No publishers; estimator never reads this file.
        p=m.pose.position;q=m.pose.orientation
        self.truth_out.write(json.dumps(dict(sim_time=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,xyz=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w]))+'\n');self.truth_out.flush()
    def cloud(self,m):
        self.cloud_count+=1
        if self.cloud_count%5:return
        xyz=point_cloud2.read_points_numpy(m,field_names=('x','y','z'),skip_nans=True)
        xyz=np.asarray(xyz).reshape(-1,3)[::(1 if os.environ.get('U_CAPTURE_CLOUD_FULL')=='1' else 31)]
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        np.savez_compressed(self.cloud_dir/('%012d.npz'%round(stamp*1e9)),xyz=xyz,stamp=stamp,frame=m.header.frame_id,receipt=self.get_clock().now().nanoseconds*1e-9)
    def write(self,m,data):
        data['stamp']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        data['frame']=m.header.frame_id
        self.out.write(json.dumps(data)+'\n');self.out.flush()
    def wheel(self,m):
        t=m.twist.twist;p=m.pose.pose.position;q=m.pose.pose.orientation
        self.write(m,dict(source='wheel',velocity=[t.linear.x,t.linear.y,t.linear.z],angular=[t.angular.x,t.angular.y,t.angular.z],position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],covariance=list(m.twist.covariance)))
    def imu(self,m):
        a=m.linear_acceleration;w=m.angular_velocity
        self.write(m,dict(source='imu',acceleration=[a.x,a.y,a.z],angular=[w.x,w.y,w.z]))
rclpy.init(args=[])
n=Recorder(sys.argv[1])
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:
    n.out.close();n.truth_out.close();n.destroy_node()
    if rclpy.ok():rclpy.shutdown()
