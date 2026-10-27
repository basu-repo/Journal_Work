"""ROS bench checks; publishers are replaced before any command is emitted."""
from pathlib import Path
import time
import unittest
from unittest.mock import Mock
import rclpy
import yaml
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from localization.fastlio_adapter import Adapter
from localization.health import Health
from scripts.hardware_watchdog import Watchdog

ROOT = Path(__file__).resolve().parents[1]


class RuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def robot(self):
        return yaml.safe_load((ROOT/'config/robot.example.yaml').read_text())

    def test_watchdog_disarm_clamp_stale_and_readiness_loss(self):
        node = Watchdog(self.robot(), False)
        node.publisher = Mock()
        try:
            node.on_enable(Bool(data=True))
            node.on_ready(Bool(data=True))
            scan = LaserScan()
            scan.ranges = [5.]
            node.on_scan(scan)
            node.on_odom(Odometry())
            command = Twist()
            command.linear.x, command.angular.z = 10., 10.
            node.on_command(command)
            node.tick()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
            node.armed = True
            node.tick()
            out = node.publisher.publish.call_args.args[0]
            self.assertEqual(out.linear.x, node.cfg['max_linear_speed'])
            self.assertEqual(out.angular.z, node.cfg['max_angular_speed'])
            node.last_command = time.monotonic()-2
            node.tick()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
            node.on_command(command)
            node.on_ready(Bool(data=False))
            node.tick()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
        finally:
            node.destroy_node()

    def test_health_requires_fresh_inputs_and_tf(self):
        robot = self.robot()
        robot['localization']['stable_seconds'] = 0.
        node = Health(robot)
        node.pub = Mock()
        try:
            node.tick()
            self.assertFalse(node.pub.publish.call_args.args[0].data)
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.header.frame_id, odom.child_frame_id = robot['odom_frame'], robot['base_frame']
            odom.pose.pose.orientation.w = 1.
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.width = grid.info.height = 1
            grid.data = [0]
            node.on_odom(odom)
            node.on_map(grid)
            tf = TransformStamped()
            tf.header.stamp = odom.header.stamp
            node.tf = Mock()
            node.tf.lookup_transform.return_value = tf
            node.tick()
            self.assertTrue(node.pub.publish.call_args.args[0].data)
            node.odom_received -= 2.
            node.tick()
            self.assertFalse(node.pub.publish.call_args.args[0].data)
        finally:
            node.destroy_node()

    def test_adapter_rejects_frame_mismatch_and_duplicate_stamp(self):
        node = Adapter(self.robot())
        node.pub, node.tf = Mock(), Mock()
        try:
            msg = Odometry()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.orientation.w = 1.
            node.on_odom(msg)
            node.pub.publish.assert_not_called()
            msg.header.frame_id, msg.child_frame_id = 'camera_init', 'body'
            node.on_odom(msg)
            node.pub.publish.assert_called_once()
            self.assertEqual(node.pub.publish.call_args.args[0].header.frame_id, 'odom_lidar')
            node.on_odom(msg)
            node.pub.publish.assert_called_once()
        finally:
            node.destroy_node()
