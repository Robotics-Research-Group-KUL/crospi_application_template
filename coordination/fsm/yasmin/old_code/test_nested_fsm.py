import rclpy

from simple_node import Node
from example_interfaces.srv import AddTwoInts

from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ServiceState
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub

import time

def init_callback(blackboard: Blackboard) -> str:
    blackboard.a = 10
    blackboard.b = 5
    print("enter state")
    time.sleep(2)
    return SUCCEED


class InnerStateMachine(StateMachine):
    def __init__(self,sm_node: Node):
        self.sm_node_ = sm_node
        super().__init__(outcomes=["finished_inner"])

        self.add_state("STATE_INNER_A", CbState([SUCCEED], init_callback),
                     transitions={SUCCEED: "STATE_INNER_B"})
        self.add_state("STATE_INNER_B", CbState([SUCCEED], init_callback),
                     transitions={SUCCEED: "finished_inner"})

class OuterStateMachine(Node):
    def __init__(self):
        super().__init__("etasl_yasmin_node")

        sm = StateMachine(outcomes=["finished_outer"])

        self.inner_sm = InnerStateMachine(self)
        # self.add_state(self.inner_sm)

        sm.add_state("STATE_OUTER_A", CbState([SUCCEED], init_callback),
                     transitions={SUCCEED: "NESTED_FSM_INNER"})
        sm.add_state("NESTED_FSM_INNER", self.inner_sm,
                    transitions={"finished_inner": "STATE_OUTER_B",
                                ABORT: "finished_outer"})
        sm.add_state("STATE_OUTER_B", CbState([SUCCEED], init_callback),
                     transitions={SUCCEED: "STATE_OUTER_A",
                                ABORT: "finished_outer"})

        # execute
        YasminViewerPub(self, "NESTED_FSM_OUTER", sm) #To visualize complete FSM
        YasminViewerPub(self, "NESTED_FSM_INNER", self.inner_sm) #To visualize inner FSM
        outcome = sm()
        print(outcome)


def main(args=None):
    # rclpy.init()
    # outer_sm = OuterStateMachine()
    # bb = Blackboard();
    # outer_sm.execute(bb)
    # rclpy.shutdown()


    print("yasmin_etasl")
    rclpy.init(args=args)
    node = OuterStateMachine()
    node.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()