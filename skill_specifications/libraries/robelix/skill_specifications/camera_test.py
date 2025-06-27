#!/usr/bin/env python3

import rclpy
import sys

from threading import Lock
import rclpy.time
from rclpy.node import Node

# from betfsm.betfsm import TickingState,TICKING,Blackboard, Sequence, Message, TickingStateMachine
from betfsm.betfsm import *
from betfsm.betfsm_node import BeTFSMNode
from betfsm.betfsm_etasl import load_task_list, eTaSL_StateMachine
from betfsm.graphviz_visitor import GraphViz_Visitor
from betfsm.logger import get_logger,set_logger
from betfsm.betfsm_ros import *


from rclpy.duration import Duration

import tf2_ros
# import PyKDL
# from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import TransformStamped
import numpy as np
import casadi as cas         
import random as rand
import json
import pickle

from scipy.spatial.transform import Rotation as R
from board_localization import calibrate





from tf_transformations import quaternion_matrix, translation_matrix, quaternion_from_matrix, translation_from_matrix, quaternion_from_euler

from betfsm.betfsm_action_server import CheckForCanceledAction

import math

class BeTFSMRunner:
    def __init__(self,node:Node, statemachine:TickingState, blackboard: Blackboard, sampletime):
        get_logger().info(f"BeTFSMRunner started with sample time {sampletime}")
        self.node  = node
        self.sm    = statemachine
        self.bm    = blackboard
        self.timer = self.node.create_timer(sampletime, self.timer_cb)
        self.outcome = "TICKING"
        self.outcome_lock = Lock()

    def timer_cb(self):
        outcome = self.sm(self.bm)
        #resetprint("---"),
        if outcome!=TICKING:
            self.timer.cancel()
            self.set_outcome(outcome)
            self.sm.reset()

    def set_outcome(self, outcome):
        with self.outcome_lock:
            self.outcome = outcome

    def get_outcome(self):
        with self.outcome_lock:
            outcome = self.outcome
        return outcome

class StorePose(Generator):
    def __init__(self):
        super().__init__("StorePose",[SUCCEED])    
        self.etasl_node = BeTFSMNode.get_instance()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.etasl_node)

    def co_execute(self,blackboard):


        if 'calibration_poses_robot' not in blackboard:
            blackboard['calibration_poses_robot'] = []

        if 'calibration_poses_camera' not in blackboard:
            blackboard['calibration_poses_camera'] = []


        # Append to blackboar
        robot_pose = self.get_tf(child_frame = "maira7M_flange", parent_frame = "maira7M_root_link")
        blackboard['calibration_poses_robot'].append(robot_pose)
        # blackboard['calibration_poses_camera'].append(blackboard['calibration_poses_robot'][-1])
        # blackboard['calibration_poses_camera'].append(self.get_random_tf())
        self.etasl_node.get_logger().info("TF stored in blackboard.")
        print(robot_pose)
        print("++++++++++++++++++++++++++++")

    def get_tf(self, child_frame: str, parent_frame: str):
        try:
            # Lookup transform (latest available)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame, 
                rclpy.time.Time(),  # Use the latest available transform
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            print("------------------Transform:-------------------")
            print("parent frame: ", parent_frame)
            print("child frame: ", child_frame)
            print(transform.transform)

            # Extract translation and rotation (quaternion)
            translation = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]

            quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]

            # Convert to homogeneous transformation matrix (4x4)
            # print("Translation matrix:")
            # print(translation_matrix(translation))

            # print("Rotation matrix:")
            # print(quaternion_matrix(quaternion))

            homogeneous_matrix = np.dot(translation_matrix(translation), quaternion_matrix(quaternion))
            # print("Transformation matrix:")
            # print(homogeneous_matrix)

            return homogeneous_matrix

        except Exception as e:
            self.etasl_node.get_logger().warn(f"Failed to lookup transform: {e}")

    def get_random_tf(self):

        # Random translation:
        translation = [
            rand.uniform(-1,1),
            rand.uniform(-1,1),
            rand.uniform(-1,1)
        ]

        # Random quaternion:
        quaternion = quaternion_from_euler(rand.uniform(-2*np.pi,2*np.pi), rand.uniform(-2*np.pi,2*np.pi), rand.uniform(-2*np.pi,2*np.pi), axes='sxyz')


        # quaternion = [
        #     quat[0],
        #     quat[1],
        #     quat[2],
        #     quat[3]
        # ]

        # Convert to homogeneous transformation matrix (4x4)
        homogeneous_matrix = np.dot(translation_matrix(translation), quaternion_matrix(quaternion))
        # print("Transformation matrix:")
        # print(homogeneous_matrix)

        return homogeneous_matrix



# main
def main(args=None):

    print("betfsm")
    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("camera_calibration")

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    #set_logger("state",my_node.get_logger())

    blackboard = {}

    load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/robelix/tasks/camera_calibration.json",blackboard)
    
    sm  = Sequence("CalibrationRoutine", children=[
                                                    # PrintEstimates(),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_1","calibration_pose_1",node=None),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_2","calibration_pose_2",node=None),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_3","calibration_pose_3",node=None),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_4","calibration_pose_4",node=None),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_5","calibration_pose_5",node=None),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_6","calibration_pose_6",node=None),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_7","calibration_pose_7",node=None),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_8","calibration_pose_8",node=None),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_9","calibration_pose_9",node=None),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_10","calibration_pose_10",node=None),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_11","calibration_pose_11",node=None),
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_12","calibration_pose_12",node=None)
    ])

    # prints a graphviz representation of sm:
    vis = GraphViz_Visitor()
    sm.accept(vis)
    vis.print()

    runner = BeTFSMRunner(my_node,sm,blackboard,0.01)

    rclpy.spin(my_node)
    
    # try:
    #     while (runner.get_outcome()==TICKING):
    #         rclpy.spin_once(my_node)
    #     rclpy.shutdown()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     print("final outcome : ",runner.get_outcome())
        
    print("shutdown")

if __name__ == "__main__":
    sys.exit(main(sys.argv))
