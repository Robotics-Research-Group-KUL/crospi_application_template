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

# from lifecycle_msgs.msg import Transition
from skill_specifications.libraries.mav_control_lib.skill_specifications.mav_actions import LCSM_MAV

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


print("betfsm")
rclpy.init(args=None)

my_node = BeTFSMNode.get_instance("mav_maira_test")

set_logger("default",my_node.get_logger())

blackboard = {}

class MavLCSM(TickingStateMachine):
    def __init__(self, name:str, mav_node:str, node:Node = None):

        super().__init__(name, outcomes=[SUCCEED, ABORT, TIMEOUT])

        self.add_state(LifeCycle(name="DeactivateMAV", srv_name=f"/{mav_node}", 
                                 transition=Transition.DEACTIVATE, node=node, 
                                 timeout=Duration(seconds=5)),
                                 transitions={SUCCEED: "CleanupMAV",
                                                ABORT: "CleanupMAV",
                                                TIMEOUT: ABORT})

        self.add_state(LifeCycle(name="CleanupMAV", srv_name=f"/{mav_node}", 
                                 transition=Transition.CLEANUP, node=node, 
                                 timeout=Duration(seconds=5)),
                                 transitions={SUCCEED: "ConfigureMAV",
                                                ABORT: "ConfigureMAV",
                                                TIMEOUT: ABORT})

        self.add_state(LifeCycle(name="ConfigureMAV", srv_name=f"/{mav_node}", 
                                 transition=Transition.CONFIGURE, node=node, 
                                 timeout=Duration(seconds=5)),
                                 transitions={SUCCEED: "ActivateMAV",
                                                ABORT: ABORT})

        self.add_state(LifeCycle(name="ActivateMAV", srv_name=f"/{mav_node}", 
                                 transition=Transition.ACTIVATE, node=node, 
                                 timeout=Duration(seconds=5)),
                                 transitions={SUCCEED: SUCCEED})


load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/robelix/tasks/mav_maira_test.json",blackboard)
sm  = Sequence("CalibrationRoutine", children=[
                                                MavLCSM(name="MAV_LCSM", mav_node="schwarzmuller_driver_lifecycle_node", node=my_node)
                                                # eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                # LCSM_MAV(mav_node="schwarzmuller_driver_lifecycle_node", transition_id=Transition.TRANSITION_ACTIVATE, node=my_node),
                                                # LifeCycle(name="DeactivateMAV", srv_name="/schwarzmuller_driver_lifecycle_node", transition=Transition.DEACTIVATE, node=my_node, timeout=Duration(seconds=5)),
                                                # LifeCycle(name="CleanupMAV", srv_name="/schwarzmuller_driver_lifecycle_node", transition=Transition.CLEANUP, node=my_node, timeout=Duration(seconds=5)),
                                                # LifeCycle(name="ConfigureMAV", srv_name="/schwarzmuller_driver_lifecycle_node", transition=Transition.CONFIGURE, node=my_node, timeout=Duration(seconds=5)),
                                                # LifeCycle(name="ActivateMAV", srv_name="/schwarzmuller_driver_lifecycle_node", transition=Transition.ACTIVATE, node=my_node, timeout=Duration(seconds=5)),
                                                # TimedWait(name="WaitForMAV", timeout=Duration(seconds=5), node=my_node),
                                                # LifeCycle(name="ActivateMAV", srv_name="/schwarzmuller_driver_lifecycle_node", transition=Transition.TRANSITION_ACTIVATE, node=my_node),
                                                # eTaSL_StateMachine("MovingToPreTest","MovingToPreTest",node=None),
                                                # eTaSL_StateMachine("MovingMairaMav","MovingMairaMav",node=None),
                                                # eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                # eTaSL_StateMachine("TestMavOrientation","TestMavOrientation",node=None),
                                                # eTaSL_StateMachine("NullspaceMairaJoystickMav","NullspaceMairaJoystickMav",node=None),
                                                # eTaSL_StateMachine("MovingMairaMavTrapezoidal","MovingMairaMavTrapezoidal",node=None),
                                                # LifeCycle(name="DeactivateMAV", srv_name="/schwarzmuller_driver_lifecycle_node", transition=Transition.DEACTIVATE, node=my_node, timeout=Duration(seconds=5))
                                                ])

# prints a graphviz representation of sm:

vis = GraphViz_Visitor()
sm.accept(vis)
vis.print()

runner = BeTFSMRunner(my_node,sm,blackboard,0.01)

try:
    rclpy.spin(my_node)
except KeyboardInterrupt:
    
    print("=-=====================keyboard interrupt, shutting down\n")