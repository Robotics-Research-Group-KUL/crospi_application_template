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

# from skill_requirements import ParamValidator as pv

# param = pv.parameters("Task description goes here", [
#     pv.p_scalar({"name": "maxvel", "description": "Maximum velocity [m/s]", "default": 0.1, "required": True, "maximum": 0.5}),
#     pv.p_scalar({"name": "maxacc", "description": "Maximum acceleration [m/s^2]", "default": 0.1, "required": True, "maximum": 0.5}),
#     pv.p_scalar({"name": "eq_r", "description": "Equivalent radius", "default": 0.08, "required": False}),
#     pv.p_string({"name": "task_frame", "description": "Name of frame used to control the robot in cartesian space", "default": "tcp_frame", "required": False}),
#     pv.p_array({"name": "delta_pos", "type": "number", "default": [0.0, 0.0, 0.0], "description": "3D array of distances [m] that the robot will move w.r.t. the starting position in the X,Y,Z coordinates", "required": True, "minimum": -1.5, "maximum": 1.5, "minItems": 3, "maxItems": 3}),
# ])


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

class MyStateMachine(TickingStateMachine):
    def __init__(self):
        super().__init__("my_state_machine",[SUCCEED, ABORT])

        self.add_state(
            eTaSL_StateMachine("SineDebug","SineDebug",node=None), 
            transitions={SUCCEED: SUCCEED, 
                        ABORT: ABORT}
        )


# main
def main(args=None):

    print("betfsm")
    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("sine_debug")

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    #set_logger("state",my_node.get_logger())

    blackboard = {}

    load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/skill_lib_example/tasks/sine_debug.json",blackboard)
    
    sm = MyStateMachine()

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
