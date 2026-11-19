#!/usr/bin/env python3
"""Copy robot TF while removing dynamic parents that compete with localization."""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfOwnerFilter(Node):
    def __init__(self, source_topic, output_topic, base_frame):
        super().__init__('hil_tf_owner_filter')
        self.base_frame = base_frame.lstrip('/')
        qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(TFMessage, output_topic, qos)
        self.subscription = self.create_subscription(
            TFMessage, source_topic, self.on_tf, qos)
        self.dropped = set()

    def on_tf(self, message):
        accepted = []
        for transform in message.transforms:
            child = transform.child_frame_id.lstrip('/')
            if child == self.base_frame:
                edge = (transform.header.frame_id, transform.child_frame_id)
                if edge not in self.dropped:
                    self.get_logger().warn(
                        f'Ignoring external dynamic TF {edge[0]} -> {edge[1]}; '
                        'HIL localization owns this child')
                    self.dropped.add(edge)
                continue
            accepted.append(transform)
        if accepted:
            self.publisher.publish(TFMessage(transforms=accepted))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-topic', required=True)
    parser.add_argument('--output-topic', required=True)
    parser.add_argument('--base-frame', required=True)
    args, ros = parser.parse_known_args()
    if args.source_topic == args.output_topic:
        parser.error('source-topic and output-topic must differ to avoid a TF loop')
    rclpy.init(args=ros)
    node = TfOwnerFilter(args.source_topic, args.output_topic, args.base_frame)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
