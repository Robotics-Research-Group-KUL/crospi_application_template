#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TransformStamped, Twist, Pose, Vector3, Point
from tf2_ros import TransformBroadcaster
from rclpy.time import Time
from tf_transformations import quaternion_from_euler



class SineTFBroadcaster(Node):
    def __init__(self):
        super().__init__('sine_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 20Hz

        self.frame_id = "base_link"
        self.child_frame_id = "sine_frame"
        self.start_time = self.get_clock().now()

        self.amplitude = 0.2  # meters
        self.frequency = 0.5  # Hz

        # Publisher for the twist
        self.twist_pub = self.create_publisher(Twist, '/etasl/feedforward/twist_of_tf', 1)
        self.velocity_pub = self.create_publisher(Vector3, '/etasl/feedforward/velocity_of_vector', 1)
        self.pose_pub = self.create_publisher(Pose, '/tf_to_follow', 1)
        self.vector_pub = self.create_publisher(Vector3, '/vector_to_follow', 1)
        self.point_pub = self.create_publisher(Point, '/point_to_follow', 1)

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9  # seconds

        omega = 2 * math.pi * self.frequency
        x = self.amplitude * math.sin(omega * elapsed)
        dx_dt = self.amplitude * omega * math.cos(omega * elapsed)
        # print(x)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = x
        t.transform.translation.y = 0.5
        t.transform.translation.z = 0.3

        #Define rotation in roll pitch yaw format and then transform to quaternion:  

        # Identity rotation (no rotation)
        roll = 0.0
        pitch = math.pi
        yaw = 0.0  # for example, a sine wave yaw

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

        # Create and publish Twist
        twist_msg = Twist()
        twist_msg.linear.x = dx_dt
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0

        # No angular velocity (rotation is static)
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        self.twist_pub.publish(twist_msg)

        # Create and publish Velocity
        velocity_msg = Vector3()
        velocity_msg.x = dx_dt
        velocity_msg.y = 0.0
        velocity_msg.z = 0.0

        self.velocity_pub.publish(velocity_msg)

        # Publish in a Pose message as well
        pose_msg = Pose()
        pose_msg.position.x = t.transform.translation.x
        pose_msg.position.y = t.transform.translation.y
        pose_msg.position.z = t.transform.translation.z
        pose_msg.orientation.x = t.transform.rotation.x
        pose_msg.orientation.y = t.transform.rotation.y
        pose_msg.orientation.z = t.transform.rotation.z
        pose_msg.orientation.w = t.transform.rotation.w

        self.pose_pub.publish(pose_msg)

        # Publish in a Vector3 message as well
        vector_msg = Vector3()
        vector_msg.x = t.transform.translation.x
        vector_msg.y = t.transform.translation.y
        vector_msg.z = t.transform.translation.z

        self.vector_pub.publish(vector_msg)
        
        # Publish in a Point message as well
        point_msg = Point()
        point_msg.x = t.transform.translation.x
        point_msg.y = t.transform.translation.y
        point_msg.z = t.transform.translation.z

        self.point_pub.publish(point_msg)




def main():
    rclpy.init()
    node = SineTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
