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

from etasl_ros2.srv import TaskSpecificationFile
from etasl_ros2.srv import TaskSpecificationString

from functools import partial

import etasl_yasmin_utils as etasl



import time

class ConfigureEtasl(ServiceState):
    def __init__(self, node: Node) -> None:
        self.node = node
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


def init_callback(blackboard: Blackboard) -> str:
    blackboard.a = 10
    blackboard.b = 5
    return SUCCEED

def test_callback(blackboard: Blackboard) -> str:
    blackboard.a = 10
    blackboard.b = 5
    print("enter state")
    time.sleep(2)
    return SUCCEED


def print_sum(blackboard: Blackboard) -> str:
    print(f"Sum: {blackboard.sum}")
    time.sleep(1)
    return SUCCEED


# def readTaskSpecificationFile(blackboard: Blackboard, node: Node, file_name: String, rel_shared_dir: bool):
#     req_task = TaskSpecificationFile.Request()
#     req_task.file_path = file_name
#     req_task.rel_shared_dir = rel_shared_dir
#     resp_task = node.etasl_file_cli.call(req_task) 
#     print("The response call is" + str(resp_task))
#     time.sleep(1)

#     return SUCCEED

# def readTaskSpecificationString(blackboard: Blackboard, node: Node, string_p: String):

#     req_task = TaskSpecificationString.Request()
#     req_task.str = string_p
#     resp_task = node.etasl_string_cli.call(req_task)
#     print("The response call is" + str(resp_task))

#     return SUCCEED

class EtaslFSMNode(Node):


    def __init__(self):
        super().__init__("yasmin_node")
        self.etasl_clients = {} #used to add service clients on the fly


        # self.etasl_file_cli = self.create_client(TaskSpecificationFile, 'etasl_node/readTaskSpecificationFile')
        # self.etasl_string_cli = self.create_client(TaskSpecificationString, 'etasl_node/readTaskSpecificationString')

        # create a state machine
        sm = StateMachine(outcomes=["finished","finished_inner"])

        # add states

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
                transitions={SUCCEED: "finished_inner",
                            ABORT: "finished"})

        task_spec_cb = partial(etasl.readTaskSpecificationFile, node=self,file_name= "move_cartesianspace.lua", rel_shared_dir=True)
        
        sm_out = StateMachine(outcomes=["finished_outer"])

        sm_out.add_state("STATE_OUTER_A", CbState([SUCCEED], task_spec_cb),
                     transitions={SUCCEED: "NESTED_FSM_INNER"})
        sm_out.add_state("NESTED_FSM_INNER", sm,
                    transitions={"finished_inner": "STATE_OUTER_B",
                                "finished": "finished_outer",
                                ABORT: "finished_outer"})
        sm_out.add_state("STATE_OUTER_B", CbState([SUCCEED], test_callback),
                     transitions={SUCCEED: "STATE_OUTER_A",
                                ABORT: "finished_outer"})


        # sm.add_state("PRINTING_SUM", CbState([SUCCEED], print_sum),
        #              transitions={SUCCEED: "finished"})

        # sm.add_state("CONFIG_ETASL", CbState([SUCCEED], init_callback),
        #              transitions={SUCCEED: "finished"})

        # pub
        YasminViewerPub(self, "Lifecycle FSM", sm)
        YasminViewerPub(self, "Complete FSM", sm_out)

        etasl.define_services(self) #Creates all the etasl service clients and adds them to self.etasl_clients 

        # execute
        outcome = sm_out() #This function will block until the output of sm_out is achieved
        print(outcome)
    
    def add_client(self, name: String, service_type):
        self.etasl_clients[name] = self.create_client(service_type, 'etasl_node/{}'.format(name))
        print("adding service: etasl_node/{}".format(name))
        return
    
    def call_service(self, srv_name: String, req_task):
        self.etasl_clients[srv_name].call(req_task)
        return 


# main
def main(args=None):

    print("yasmin_etasl")
    rclpy.init(args=args)
    node = EtaslFSMNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
