#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


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

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# from lifecycle_msgs.srv import ChangeState_Response

from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


import time

class ConfigureEtasl(ServiceState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            node,  # node
            ChangeState,  # srv type
            "/etasl_node/change_state",  # service name
            self.create_request_handler,  # cb to create the request
            [SUCCEED],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> ChangeState.Request:

        req = ChangeState.Request()
        req.transition.id=1
        req.transition.label='configure'
        print("ConfigureEtasl")
        return req

    def response_handler(self,blackboard: Blackboard,response: ChangeState.Response) -> str:

        print("Service response success: " + str(response.success))
        blackboard.success = response.success
        time.sleep(1)
        return SUCCEED

class ActivateEtasl(ServiceState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            node,  # node
            ChangeState,  # srv type
            "/etasl_node/change_state",  # service name
            self.create_request_handler,  # cb to create the request
            [SUCCEED],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> ChangeState.Request:

        req = ChangeState.Request()
        req.transition.id=1
        req.transition.label='activate'
        print("ActivateEtasl")
        return req

    def response_handler(self,blackboard: Blackboard,response: ChangeState.Response) -> str:

        print("Service response success: " + str(response.success))
        blackboard.success = response.success
        return SUCCEED

class DeactivateEtasl(ServiceState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            node,  # node
            ChangeState,  # srv type
            "/etasl_node/change_state",  # service name
            self.create_request_handler,  # cb to create the request
            [SUCCEED],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> ChangeState.Request:

        req = ChangeState.Request()
        req.transition.id=1
        req.transition.label='deactivate'
        print("DeactivateEtasl")
        return req

    def response_handler(self,blackboard: Blackboard,response: ChangeState.Response) -> str:

        print("Service response success: " + str(response.success))
        blackboard.success = response.success
        time.sleep(1)
        return SUCCEED

class CleanupEtasl(ServiceState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            node,  # node
            ChangeState,  # srv type
            "/etasl_node/change_state",  # service name
            self.create_request_handler,  # cb to create the request
            [SUCCEED],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> ChangeState.Request:

        req = ChangeState.Request()
        req.transition.id=1
        req.transition.label='cleanup'
        print("CleanupEtasl")
        return req

    def response_handler(self,blackboard: Blackboard,response: ChangeState.Response) -> str:

        print("Service response success: " + str(response.success))
        blackboard.success = response.success
        time.sleep(1)
        return SUCCEED


class WaitingEtasl(MonitorState):
    def __init__(self, node: Node) -> None:
        super().__init__(node,  # node
                         String,  # msg type
                         "etasl/events",  # topic name
                         [SUCCEED,"outcome_continue"],  # outcomes
                         self.monitor_handler,  # monitor handler callback
                         qos=qos_profile_sensor_data,  # qos for the topic sbscription
                         msg_queue=10,  # queue of the monitor handler callback
                         timeout=None  # timeout to wait for msgs in seconds
                                     # if not None, CANCEL outcome is added
                         )

    def monitor_handler(self, blackboard: Blackboard, msg: String) -> str:
        # print(msg.data)
        if msg.data == "e_finished@etasl_node":
            return SUCCEED

        return "outcome_continue"


class AddTwoIntsState(ServiceState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            node,  # node
            AddTwoInts,  # srv type
            "/add_two_ints",  # service name
            self.create_request_handler,  # cb to create the request
            ["outcome1"],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> AddTwoInts.Request:

        req = AddTwoInts.Request()
        req.a = blackboard.a
        req.b = blackboard.b
        print("AddTwoIntsState")
        time.sleep(1)
        return req

    def response_handler(self, blackboard: Blackboard, response: AddTwoInts.Response) -> str:

        blackboard.sum = response.sum
        return "outcome1"


def init_callback(blackboard: Blackboard) -> str:
    blackboard.a = 10
    blackboard.b = 5
    return SUCCEED


def print_sum(blackboard: Blackboard) -> str:
    print(f"Sum: {blackboard.sum}")
    time.sleep(1)
    return SUCCEED


class ServiceClientDemoNode(Node):

    def __init__(self):
        super().__init__("yasmin_node")

        # create a state machine
        sm = StateMachine(outcomes=["finished"])

        # add states
        sm.add_state("INIT", CbState([SUCCEED], init_callback),
                     transitions={SUCCEED: "CONFIG_ETASL"})

        sm.add_state("CONFIG_ETASL", ConfigureEtasl(self),
                transitions={SUCCEED: "ACTIVATE_ETASL",
                            ABORT: "finished"})
        sm.add_state("ACTIVATE_ETASL", ActivateEtasl(self),
                transitions={SUCCEED: "WAITING_ETASL",
                            ABORT: "finished"})
        sm.add_state("WAITING_ETASL", WaitingEtasl(self),
                transitions={SUCCEED: "DEACTIVATE_ETASL",
                            "outcome_continue": "WAITING_ETASL",
                            ABORT: "finished"})
        sm.add_state("DEACTIVATE_ETASL", DeactivateEtasl(self),
                transitions={SUCCEED: "CLEANUP_ETASL",
                            ABORT: "finished"})
        sm.add_state("CLEANUP_ETASL", CleanupEtasl(self),
                transitions={SUCCEED: "CONFIG_ETASL",
                            ABORT: "finished"})

        # sm.add_state("PRINTING_SUM", CbState([SUCCEED], print_sum),
        #              transitions={SUCCEED: "finished"})

        # sm.add_state("CONFIG_ETASL", CbState([SUCCEED], init_callback),
        #              transitions={SUCCEED: "finished"})

        # pub
        YasminViewerPub(self, "YASMIN_ETASL", sm)

        # execute
        outcome = sm()
        print(outcome)


# main
def main(args=None):

    print("yasmin_etasl")
    rclpy.init(args=args)
    node = ServiceClientDemoNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
