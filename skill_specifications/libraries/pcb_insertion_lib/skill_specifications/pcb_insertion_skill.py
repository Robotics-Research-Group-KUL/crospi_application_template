import rclpy
import sys

from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *

import math

from std_srvs.srv import Empty
from geometry_msgs.msg import Wrench

from dualquaternion import quat
from dualquaternion import SO3, SE3

import numpy as np
import pdb
import os
sm_path = os.path.dirname(os.path.abspath(__file__))
from rclpy.qos import QoSProfile
import json
from schunk_control_egh80.srv import ControlEGH80
from pickit_comm import PickitInterface

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

    def set_outcome(self, outcome):
        with self.outcome_lock:
            self.outcome = outcome

    def get_outcome(self):
        with self.outcome_lock:
            outcome = self.outcome
        return outcome

class ConfigurePickit(Generator):
    """
    Configure the Pickit interface with the given parameters and store it in the blackboard under the given key.
    """
    def __init__(self, ip:str, port: int, setup_id: int, product_id: int, mult: int, bb_key_interface:str):
        super().__init__("ConfigurePickit",[SUCCEED, CANCEL])
        self.ip = ip
        self.port = port
        self.setup_id = setup_id
        self.product_id = product_id
        self.mult = mult
        self.bb_key_interface = bb_key_interface

    def co_execute(self,blackboard):
        self.pickitI = PickitInterface(self.ip, self.port, self.mult, non_blocking=True)
        blackboard[self.bb_key_interface] = self.pickitI
        yield TICKING
        self.pickitI.configure_environment(self.setup_id, self.product_id)
        while not self.pickitI.is_response_ready():
            yield TICKING
        succeed = self.pickitI.get_configure_environment_result()
        if succeed:
            print("[Pickit] Environment Configured")
            yield SUCCEED
        else:
            print("[Pickit] Error configuring environment")
            yield CANCEL

class CaptureImage(Generator):
    """
    Capture an image from the Pickit interface.
    """
    def __init__(self, bb_key_interface:str):
        super().__init__("CaptureAndProcess",[SUCCEED, CANCEL])
        self.bb_key_interface = bb_key_interface

    def co_execute(self,blackboard):
        pickitI = blackboard[self.bb_key_interface]
        pickitI.capture_image()
        while not pickitI.is_response_ready():
            yield TICKING
        succeed = pickitI.get_capture_image_result()
        if succeed:
            print("[Pickit] Imaged caputured")
            yield SUCCEED
        else:
            print("[Pickit] Error capturing image")
            yield CANCEL

class DetectObjects(Generator):
    """
    Detect objects in the last image captured by the Pickit interface.
    """
    def __init__(self, bb_key_object:str, bb_key_interface:str):
        super().__init__("DetectObjects",[SUCCEED, CANCEL])
        self.bb_key_object = bb_key_object
        self.bb_key_interface = bb_key_interface

    def co_execute(self,blackboard):
        pickitI = blackboard[self.bb_key_interface]
        pickitI.process_image()
        while not pickitI.is_response_ready():
            yield TICKING
        response = pickitI.get_process_image_result()
        if response["object_found"]:
            blackboard[self.bb_key_object] = response
            print("Object Detected: ", response)
            yield SUCCEED
        else:
            yield CANCEL
    

class ChangePickitSetup(Generator):
    """
    Change the Pickit setup and product to the given setup_id and product_id respectively.
    """
    def __init__(self, bb_key_interface:str, setup_id: int, product_id: int):
        super().__init__("ChangePickitSetup",[SUCCEED])
        self.bb_key_interface = bb_key_interface
        self.setup_id = setup_id
        self.product_id = product_id

    def co_execute(self,blackboard):
        pickitI = blackboard[self.bb_key_interface]
        pickitI.configure_environment(self.setup_id, self.product_id)
        yield SUCCEED
   
def get_objects_pose(pickit_ip, pickit_port, pickit_setup_id, pickit_object_dict, pickit_mult, node=None):
    """
    Sequence of tasks to get the pose of the object in the Pickit interface.
    pickit_object_dict: dictionary with the object name as key and the object id in the Pickit interface as value.
    """
    configured_and_image_captured = False
    state_list = []
    for object_key, object_id in pickit_object_dict.items():
        if not configured_and_image_captured:
            state_list.append(ConfigurePickit(pickit_ip, pickit_port, pickit_setup_id, object_id, pickit_mult, "pickit_interface"))
            state_list.append(CaptureImage("pickit_interface"))
            configured_and_image_captured = True
        else:
            state_list.append(ChangePickitSetup("pickit_interface", pickit_setup_id, object_id))
        state_list.append(DetectObjects(object_key, "pickit_interface"))
    return Sequence("ConfigureCaptureandDetect", children=state_list)

def insertion_FSM(node=None):

    return None

def application_FSM(sm, node=None):
    
    return ConcurrentSequence("test", [eTaSL_StateMachine("MovingHome","MovingHome",node=node), sm])

def main(args=None):
    pickit_mult = 10000  # Scale factor for position and orientation values
    pickit_ip = "169.254.5.180"
    pickit_port = 5001
    pickit_setup_id = 24
    pickit_PCB_id = 46
    pickit_fixture_id = 47
    pickit_object_dict = {"PCB": pickit_PCB_id, "Fixture": pickit_fixture_id}

    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("pcb_insertion_skill")

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    #set_logger("state",my_node.get_logger())

    blackboard = {}

    load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/pcb_insertion_lib/tasks/pcb_insertion_skill.json", blackboard)

    # TODO Change product id to a list that takes multiple and loop through them
    sm_objects = get_objects_pose(pickit_ip, pickit_port, pickit_setup_id, pickit_object_dict, pickit_mult, my_node)
    sm = application_FSM(sm_objects, my_node)
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