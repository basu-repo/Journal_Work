#!/usr/bin/env python3
"""Robot command watchdog and independent simulation telemetry."""
import argparse,json,time,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener,TransformException
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry,Path as PathMsg
from robot.path_history import PathRecorder
from ros_gz_interfaces.msg import Contacts
import yaml


class Runtime(Node):
    def __init__(self,config,output):
        super().__init__('robot_runtime')
        self.cfg=config;self.output=output;self.enabled=False;self.last_cmd=0.;self.scan_time=0.;self.odom_time=0.;self.cmd=Twist();self.last_contact={};self.last_truth=0.;self.last_enable=0.;self.last_log=0.;self.previous_output=None
        self.localization_ok=False;self.localization_time=0.
        self.path_recorder=PathRecorder(output)
        for kind,topic in config.get('path_topics',{}).items():
            self.create_subscription(PathMsg,topic,lambda m,k=kind,t=topic:self.path_recorder.record(k,t,m,self.get_clock().now().nanoseconds*1e-9),10)
        self.tf=Buffer();self.tf_listener=TransformListener(self.tf,self);self.last_pose_log=0.
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool,'/localization/ready',self.localization,qos)
        self.create_subscription(Bool,'/navigation/enabled',self.enable,qos)
        self.create_subscription(Twist,'/cmd_vel',self.command,10)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,config.get('odom_topic','/odom'),lambda m:setattr(self,'odom_time',time.monotonic()),qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/simulation/ground_truth/pose',self.truth,qos_profile_sensor_data)
        for link in config['contact_links']:
            self.create_subscription(Contacts,'/simulation/contacts/'+link,lambda m,l=link:self.contact(l,m),qos_profile_sensor_data)
        self.pub=self.create_publisher(Twist,'/robot/cmd_vel',10)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def localization(self,msg):self.localization_ok=msg.data;self.localization_time=time.monotonic()
    def enable(self,msg):self.enabled=msg.data;self.last_enable=time.monotonic()
    def command(self,msg):self.cmd=msg;self.last_cmd=time.monotonic()
    def scan(self,msg):
        if len(msg.ranges):self.scan_time=time.monotonic()
    def tick(self):
        now=time.monotonic();out=Twist()
        localization_ok=self.cfg.get('localization_mode')!='gnss' or (self.localization_ok and now-self.localization_time<1.)
        # /navigation/enabled is transient-local state, not a heartbeat.
        # Fresh command, localization, scan and odometry timestamps provide
        # the fail-safe watchdog and force zero output when a producer stalls.
        if localization_ok and self.enabled and all(math.isfinite(v) for v in (self.cmd.linear.x,self.cmd.angular.z)) and now-self.last_cmd<self.cfg['command_timeout'] and now-self.scan_time<1.0 and now-self.odom_time<1.0:
            out.linear.x=max(-self.cfg['reverse_speed'],min(self.cfg['max_linear_speed'],self.cmd.linear.x))
            out.angular.z=max(-self.cfg['max_angular_speed'],min(self.cfg['max_angular_speed'],self.cmd.angular.z))
        self.pub.publish(out)
        if now-self.last_pose_log>=.2:
            self.last_pose_log=now; transforms={}
            odom=self.cfg.get('odom_frame','odom')
            for parent,child in [('map',self.cfg['base_frame']),(odom,self.cfg['base_frame']),('map',odom)]:
                try:
                    tf=self.tf.lookup_transform(parent,child,Time());p=tf.transform.translation;q=tf.transform.rotation
                    transforms[parent+'->'+child]={'stamp':tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9,'xyz':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w]}
                except TransformException:pass
            if transforms:
                with (self.output/'localization.jsonl').open('a') as f:f.write(json.dumps({'sim_time':self.get_clock().now().nanoseconds*1e-9,'transforms':transforms})+'\n')
        values=(out.linear.x,out.angular.z)
        if values!=self.previous_output or now-self.last_log>.5:
            self.previous_output=values;self.last_log=now
            with (self.output/'commands.jsonl').open('a') as f:f.write(json.dumps({'wall_time':time.time(),'sim_time':self.get_clock().now().nanoseconds*1e-9,'enabled':self.enabled,'linear':values[0],'angular':values[1]})+'\n')
    def truth(self,msg):
        now=time.monotonic()
        if now-self.last_truth<.2:return
        self.last_truth=now;p=msg.pose.position;q=msg.pose.orientation
        with (self.output/'truth.jsonl').open('a') as f:f.write(json.dumps({'sim_time':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'xyz':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w]})+'\n')
    def contact(self,link,msg):
        now=time.monotonic()
        if not msg.contacts or now-self.last_contact.get(link,0)<.5:return
        self.last_contact[link]=now
        pairs=[[c.collision1.name,c.collision2.name] for c in msg.contacts]
        with (self.output/'contacts.jsonl').open('a') as f:f.write(json.dumps({'sim_time':self.get_clock().now().nanoseconds*1e-9,'link':link,'pairs':pairs})+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args,ros=parser.parse_known_args();rclpy.init(args=ros);node=Runtime(yaml.safe_load(args.config.read_text()),args.output)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist());node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
