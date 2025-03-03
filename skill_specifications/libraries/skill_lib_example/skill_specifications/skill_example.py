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

        # self.add_state(
        #     TimedWait("A",Duration(seconds=1.0)),             
        #     transitions={SUCCEED: "MovingDown", 
        #                 ABORT: ABORT})

        self.add_state(
            eTaSL_StateMachine("MovingHome","MovingHome",node=None), 
            transitions={SUCCEED: "MovingDown", 
                        ABORT: ABORT}
        )

        self.add_state(
            eTaSL_StateMachine("MovingDown","MovingDown",node=None), 
            transitions={SUCCEED: "MovingUp", 
                        ABORT: ABORT}
        )

        self.add_state(
            eTaSL_StateMachine("MovingUp","MovingUp",node=None), 
            transitions={SUCCEED: "MovingSpline", 
                        ABORT: ABORT}
        )

        self.add_state(
            eTaSL_StateMachine("MovingSpline","MovingSpline",node=None), 
            transitions={SUCCEED: "MovingHome", 
                        ABORT: ABORT}
        )

    #         sm_out.add_state("MovingHome", etasl_utils.nested_etasl_state(name="MovingHome",  display_in_viewer=True),
    #                 transitions={SUCCEED: "MovingDown", 
    #                              ABORT: ABORT})

    # sm_out.add_state("MovingDown", etasl_utils.nested_etasl_state(name="MovingDown",  display_in_viewer=True),
    #                 transitions={SUCCEED: "MovingUp", 
    #                              ABORT: ABORT})
    
    # sm_out.add_state("MovingUp", etasl_utils.nested_etasl_state(name="MovingUp",  display_in_viewer=True),
    #             transitions={SUCCEED: "MovingDown", 
    #                              ABORT: ABORT})

        # self.add_state(
        #     Sequence("B", [
        #         TimedWait("a",Duration(seconds=1.0)), 
        #         TimedWait("b",Duration(seconds=1.0)), 
        #         TimedWait("c",Duration(seconds=1.0))
        #     ]), 
        #     transitions={SUCCEED:"C"}
        # )

        # self.add_state(TimedWait("C",Duration(seconds=1.0)), transitions={SUCCEED:"A"})


# main
def main(args=None):

    print("betfsm")
    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("skill_example")

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    #set_logger("state",my_node.get_logger())

    blackboard = {}

    load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/skill_lib_example/tasks/skill_example.json",blackboard)
    
 
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
