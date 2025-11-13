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


#from betfsm.graphviz_visitor import *


from betfsm.betfsm_action_server import CheckForCanceledAction

import math

import subprocess
import os
from datetime import datetime
import signal
from pathlib import Path

from ai_prism_gui import GUI
from ai_prism_gui.GuiState import GuiState

# from skill_requirements import ParamValidator as pv

# param = pv.parameters("Task description goes here", [
#     pv.p_scalar({"name": "maxvel", "description": "Maximum velocity [m/s]", "default": 0.1, "required": True, "maximum": 0.5}),
#     pv.p_scalar({"name": "maxacc", "description": "Maximum acceleration [m/s^2]", "default": 0.1, "required": True, "maximum": 0.5}),
#     pv.p_scalar({"name": "eq_r", "description": "Equivalent radius", "default": 0.08, "required": False}),
#     pv.p_string({"name": "task_frame", "description": "Name of frame used to control the robot in cartesian space", "default": "tcp_frame", "required": False}),
#     pv.p_array({"name": "delta_pos", "type": "number", "default": [0.0, 0.0, 0.0], "description": "3D array of distances [m] that the robot will move w.r.t. the starting position in the X,Y,Z coordinates", "required": True, "minimum": -1.5, "maximum": 1.5, "minItems": 3, "maxItems": 3}),
# ])

class RosbagRecorder:
    def __init__(self):
        # self.topics = topics
        self.process = None
        self.output_file_name = None

    def start_recording(self, topics, output_file_name=None):
        if self.process:
            print("Already recording.")
            return

        if output_file_name is None:
            output_file_name = datetime.now().strftime("rosbag_%Y%m%d_%H%M%S")

        # Ensure we don’t overwrite existing bags
        output_file_name = self._generate_unique_name(output_file_name)
        self.output_file_name = output_file_name

        cmd = ["ros2", "bag", "record", "-o", output_file_name] + topics
        self.process = subprocess.Popen(cmd)
        get_logger().info(f"ROSBAG recording STARTED with output filename: {output_file_name}")



    def stop_recording(self):
        if not self.process:
            get_logger().info("ROSBAG recording could not be stopped because it was not running.")
            return
        self.process.send_signal(signal.SIGINT)
        self.process.wait()
        get_logger().info(f"ROSBAG recording STOPED with output filename: {self.output_file_name}")

        self.process = None
        self.output_file_name = None
        

    def _generate_unique_name(self, base_name: str) -> str:
        """
        Check for existing rosbag directories/files with the given base_name.
        Always start numbering from _1, even if base_name itself doesn't exist.
        """
        base = Path(base_name)
        parent = base.parent if base.parent != Path('') else Path('.')
        stem = base.stem

        # Collect all names starting with the same stem
        existing = [p.name for p in parent.iterdir() if p.name.startswith(stem)]

        # Extract numeric suffixes (e.g., _1, _2, ...)
        indices = []
        for name in existing:
            suffix = name[len(stem):]
            if suffix.startswith("_") and suffix[1:].isdigit():
                indices.append(int(suffix[1:]))

        next_index = max(indices, default=0) + 1
        new_name = f"{stem}_{next_index}"

        return str(parent / new_name)


class StartRecording(Generator):
    def __init__(self, rosbag_recorder, topics, output_file_name=None):
        super().__init__("StartRecording",[SUCCEED])    
        # self.etasl_node = BeTFSMNode.get_instance()
        self.rosbag_recorder = rosbag_recorder
        self.topics = topics
        self.output_file_name = output_file_name


    def co_execute(self,blackboard):
        print("Starting recording...")
        get_logger().info(f"Starting recording with name: {self.output_file_name}")
        self.rosbag_recorder.start_recording(topics=self.topics, output_file_name=self.output_file_name)
        get_logger().info("Recording started...")

        
        yield SUCCEED

class StopRecording(Generator):
    def __init__(self, rosbag_recorder):
        super().__init__("StopRecording",[SUCCEED])    
        # self.etasl_node = BeTFSMNode.get_instance()
        self.rosbag_recorder = rosbag_recorder

    def co_execute(self,blackboard):
        self.rosbag_recorder.stop_recording()
        
        yield SUCCEED


# class RosbagRecorder(Generator):
#     def __init__(self, topics, output_file_name=None):
#         super().__init__("RosbagRecorder",[SUCCEED])    
#         self.etasl_node = BeTFSMNode.get_instance()
#         self.topics = topics
#         self.process = None
#         self.output_file_name = output_file_name


#     def co_execute(self,blackboard):


#         # if 'calibration_poses_robot' not in blackboard:
#         #     blackboard['calibration_poses_robot'] = []

#         # if 'calibration_poses_camera' not in blackboard:
#         #     blackboard['calibration_poses_camera'] = []
#         print("Starting recording...")
#         self.start_recording(output_file_name = self.output_file_name)
#         print("Recording started...")



#         # pdb.set_trace()
        
#         yield SUCCEED

#     def start_recording(self, output_file_name=None):
#         if self.process:
#             print("Already recording.")
#             return
#         if output_file_name is None:
#             output_file_name = datetime.now().strftime("rosbag_%Y%m%d_%H%M%S")
#         cmd = ["ros2", "bag", "record", "-o", output_file_name] + self.topics
#         self.process = subprocess.Popen(cmd)
#         print(f"Recording started: {output_file_name}")

#     def stop_recording(self):
#         if not self.process:
#             print("Not recording.")
#             return
#         self.process.send_signal(signal.SIGINT)
#         self.process.wait()
#         print("Recording stopped.")
#         self.process = None



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


# main
def main(args=None):

    print("betfsm")
    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("skill_example")

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    #set_logger("state",my_node.get_logger())

    blackboard = {}

    load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/skill_lib_example/tasks/ai_prism_gui_test.json",blackboard)

    rosbag_rec = RosbagRecorder()
    
    # sm = MyStateMachine()
    sm  = Sequence("CalibrationRoutine", children=[
                                                # eTaSL_StateMachine("MovingHome","MovingHome",node=my_node),
                                                eTaSL_StateMachine("MovingPreTest","MovingPreTest",node=my_node),
                                                # StartRecording(rosbag_recorder=rosbag_rec,topics=["/joint_states", "/tf", "tf_pose"], output_file_name="demo_one"),
                                                # eTaSL_StateMachine("TestPose","TestPose",node=my_node),
                                                eTaSL_StateMachine("learning_from_demonstrations","learning_from_demonstrations",node=my_node),
                                                eTaSL_StateMachine("learning_from_demonstrations_reversed","learning_from_demonstrations_reversed",node=my_node),
                                                eTaSL_StateMachine("learning_from_demonstrations","learning_from_demonstrations",node=my_node),
                                                eTaSL_StateMachine("learning_from_demonstrations_reversed","learning_from_demonstrations_reversed",node=my_node),
                                                # eTaSL_StateMachine("DemonstrationWithJoystick","DemonstrationWithJoystick",node=my_node),
                                                # eTaSL_StateMachine("MovingDown","MovingDown",node=my_node),
                                                # StopRecording(rosbag_recorder=rosbag_rec),
                                                # StartRecording(rosbag_recorder=rosbag_rec,topics=["/joint_states", "/tf"], output_file_name="demo_two"),
                                                # eTaSL_StateMachine("MovingUp","MovingUp",node=my_node),
                                                # eTaSL_StateMachine("MovingSpline","MovingSpline",node=my_node),
                                                # StopRecording(rosbag_recorder=rosbag_rec),

                                                # eTaSL_StateMachine("MovingMairaMavTrapezoidal","MovingMairaMavTrapezoidal",node=None),
                                                # LifeCycle(name="DeactivateMAV", srv_name="/schwarzmuller_driver_lifecycle_node", transition=Transition.DEACTIVATE, node=my_node, timeout=Duration(seconds=5))
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
