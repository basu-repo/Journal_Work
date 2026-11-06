#!/usr/bin/env python3
"""Isolated FAST-LIO2 frame adapter and performance recorder."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster


class Adapter(Node):
    def __init__(self, output):
        super().__init__('fastlio_navigation_adapter')
        self.output=output;self.sim_time=None;self.lidar_count=0;self.odom_count=0
        self.scan_count=0;self.map_count=0
        self.last_lidar_stamp=None;self.last_odom_stamp=None;self.last_perf=time.monotonic()
        self.stream=(output/'fastlio_navigation.jsonl').open('a')
        self.perf=(output/'fastlio_performance.jsonl').open('a')
        self.pub=self.create_publisher(Odometry,'/fastlio/odometry_base',qos_profile_sensor_data)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(Clock,'/clock',self.on_clock,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/lidar/points',self.on_lidar,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',lambda _:setattr(self,'scan_count',self.scan_count+1),qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',lambda _:setattr(self,'map_count',self.map_count+1),10)
        self.create_subscription(Odometry,'/fastlio/odometry',self.on_odom,qos_profile_sensor_data)
        self.create_timer(1.0,self.record_performance)

    def on_clock(self,msg):self.sim_time=msg.clock.sec+msg.clock.nanosec*1e-9
    def on_lidar(self,msg):
        self.lidar_count+=1;self.last_lidar_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    def on_odom(self,msg):
        self.odom_count+=1;stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.last_odom_stamp=stamp
        out=Odometry();out.header=msg.header;out.header.frame_id='odom_lidar'
        out.child_frame_id='base_link';out.pose=msg.pose;out.twist=msg.twist
        self.pub.publish(out)
        tf=TransformStamped();tf.header=out.header;tf.child_frame_id='base_link'
        p=out.pose.pose.position;q=out.pose.pose.orientation
        tf.transform.translation.x=p.x;tf.transform.translation.y=p.y;tf.transform.translation.z=p.z
        tf.transform.rotation=q;self.tf.sendTransform(tf)
        row={'stamp':stamp,'receipt_sim_time':self.sim_time,
             'message_age_s':None if self.sim_time is None else self.sim_time-stamp,
             'source_frame':msg.header.frame_id,'source_child':msg.child_frame_id,
             'target_frame':'odom_lidar','target_child':'base_link',
             'xyz':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w]}
        self.stream.write(json.dumps(row)+'\n');self.stream.flush()

    def record_performance(self):
        now=time.monotonic();dt=now-self.last_perf;self.last_perf=now
        row={'sim_time':self.sim_time,'wall_interval_s':dt,
             'lidar_count':self.lidar_count,'odom_count':self.odom_count,
             'scan_count':self.scan_count,'map_count':self.map_count,
             'lidar_stamp':self.last_lidar_stamp,'odom_stamp':self.last_odom_stamp,
             'queue_gap':self.lidar_count-self.odom_count}
        self.perf.write(json.dumps(row)+'\n');self.perf.flush()

    def destroy_node(self):
        self.stream.close();self.perf.close();super().destroy_node()


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    args,ros=p.parse_known_args();args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=ros);node=Adapter(args.output)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
