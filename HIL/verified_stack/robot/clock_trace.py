"""Record a sampled mapping between Gazebo simulation time and host UTC time."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import TimeReference


class ClockTrace(Node):
    def __init__(self, output):
        super().__init__('clock_trace')
        self.stream=(output/'clock_sync.jsonl').open('a')
        self.last_sample_ns=0
        self.publisher=self.create_publisher(TimeReference,'/recording/wall_clock',10)
        self.subscription=self.create_subscription(Clock,'/clock',self.sample,qos_profile_sensor_data)

    def sample(self, message):
        mono_ns=time.monotonic_ns()
        if mono_ns-self.last_sample_ns<1_000_000_000:return
        self.last_sample_ns=mono_ns
        wall_ns=time.time_ns()
        sec,nanosec=divmod(wall_ns,1_000_000_000)
        reference=TimeReference()
        reference.header.stamp=message.clock
        reference.time_ref.sec=sec
        reference.time_ref.nanosec=nanosec
        reference.source='host_utc_receive_time'
        self.publisher.publish(reference)
        self.stream.write(json.dumps({'sim_time_ns':message.clock.sec*1_000_000_000+message.clock.nanosec,
                                      'wall_unix_ns':wall_ns,'monotonic_ns':mono_ns})+'\n')
        self.stream.flush()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args,ros=parser.parse_known_args()
    rclpy.init(args=ros)
    node=ClockTrace(args.output)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.stream.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
