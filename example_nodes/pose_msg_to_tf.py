#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import TransformBroadcaster
from rclpy.time import Time
# from tf_transformations import quaternion_from_euler



class PoseToTFBroadcaster(Node):
    def __init__(self):
        super().__init__('PoseToTFBroadcaster')
        self.br = TransformBroadcaster(self)

        self.frame_id = "pose_frame"
        self.child_frame_id = "world"


        self.subscription = self.create_subscription(
            Pose,
            '/charuco_detector/pose',
            self.listener_callback,
            1)
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

        self.br.sendTransform(tf)



def main():
    rclpy.init()
    node = PoseToTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
