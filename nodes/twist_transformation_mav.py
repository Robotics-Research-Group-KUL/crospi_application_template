#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TransformStamped, Pose
from rclpy.time import Time
# from tf_transformations import quaternion_from_euler

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class TwistTransformationMav(Node):
    def __init__(self):
        super().__init__('TwistTransformationMav')
        self.br = TransformBroadcaster(self)

        self.frame_id = "world"
        # self.frame_id = "maira7M_root_link"
        self.child_frame_id = "board"

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # typical depth for sensor data
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=rclpy.duration.Duration(seconds=0),  # messages never expire
            deadline=rclpy.duration.Duration(seconds=0)  # no deadline
            # liveliness=rclpy.qos.LivelinessPolicy.SYSTEM_DEFAULT,
            # liveliness_lease_duration=rclpy.duration.Duration(seconds=0),
            # avoid_ros_namespace_conventions=False,
        )


        self.subscription = self.create_subscription(
            Pose,
            '/charuco_detector/pose_world_board',
            # '/charuco_detector/pose',
            self.listener_callback,
            sensor_qos)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, pose_msg):
        # self.get_logger().info('I heard: "%s"' % pose_msg.data)
        now = self.get_clock().now()
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.child_frame_id

        tf.transform.translation.x = pose_msg.position.x
        tf.transform.translation.y = pose_msg.position.y
        tf.transform.translation.z = pose_msg.position.z
        tf.transform.rotation.x = pose_msg.orientation.x
        tf.transform.rotation.y = pose_msg.orientation.y
        tf.transform.rotation.z = pose_msg.orientation.z
        tf.transform.rotation.w = pose_msg.orientation.w
        print("Publishing TF from " + self.frame_id + " to " + self.child_frame_id)

        self.br.sendTransform(tf)



def main():
    rclpy.init()
    node = TwistTransformationMav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
