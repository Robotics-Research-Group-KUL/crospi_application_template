#!/usr/bin/env python3


# import rclpy

from rclpy.node import Node
from yasmin_ros.yasmin_node import YasminNode
# from simple_node import Node
from yasmin import Blackboard
from yasmin import StateMachine


from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT
from yasmin_ros import ServiceState

from yasmin_viewer import YasminViewerPub


from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# from lifecycle_msgs.srv import ChangeState_Response

from std_msgs.msg import String
from etasl_interfaces.srv import TaskSpecificationFile
from etasl_interfaces.srv import TaskSpecificationString

from functools import partial

from typing import List, Callable, Union, Type

from event_state import EventState



import time


class ServiceManager():

    def __init__(self) -> None:
        self.node = YasminNode.get_instance()
        self.etasl_clients = {} #used to add service clients on the fly
    
    def add_client(self, name: str, service_type):
        self.etasl_clients[name] = self.node.create_client(service_type, 'etasl_node/{}'.format(name))
        self.node.get_logger().info("adding service: etasl_node/{}".format(name))
        return
    
    def call_service(self, srv_name: str, req_task):
        
        return self.etasl_clients[srv_name].call(req_task)


    def define_services(self):
        # node.add_client("configure", ChangeState)
        # node.add_client("cleanup", ChangeState)
        # node.add_client("activate", ChangeState)
        # node.add_client("deactivate", ChangeState)
        self.add_client("change_state", ChangeState)
        self.add_client("readTaskSpecificationFile", TaskSpecificationFile)
        self.add_client("readTaskSpecificationString", TaskSpecificationString)

        #Add all the etasl services here


    def readTaskSpecificationFile(self, blackboard: Blackboard, file_name: String):
        req_task = TaskSpecificationFile.Request()
        req_task.file_path = file_name
        repl = self.call_service('readTaskSpecificationFile', req_task)
        # time.sleep(1)
        # print(repl)
        # print("readTaskSpecificationFile")

        return repl

    def readTaskSpecificationString(self, blackboard: Blackboard, string_p: String):

        req_task = TaskSpecificationString.Request()
        req_task.str = string_p
        repl = self.call_service('readTaskSpecificationString', req_task)
        # print(repl)
        # print("readTaskSpecificationString")

        return repl

    def configure(self):
        req = ChangeState.Request()
        req.transition.id = 1 #I don't think that the id makes any difference
        req.transition.label = "configure"
        repl = self.call_service('change_state', req)
        # print(repl)
        # print("configuring")
        return repl

    def cleanup(self):
        req = ChangeState.Request()
        req.transition.id = 2 #I don't think that the id makes any difference
        req.transition.label = "cleanup"
        repl = self.call_service('change_state', req)
        # print("cleanup")

        return repl

    def activate(self):
        req = ChangeState.Request()
        req.transition.id = 3 #I don't think that the id makes any difference
        req.transition.label = "activate"
        repl = self.call_service('change_state', req)
        # print("activate")

        return repl
        
    def deactivate(self):
        # print("===============")
        req = ChangeState.Request()
        req.transition.id = 4 #I don't think that the id makes any difference
        req.transition.label = "deactivate"
        repl = self.call_service('change_state', req)
        # print("deactivate")

        return repl
    


class ConfigureEtasl(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            ChangeState,  # srv type
            "/etasl_node/change_state",  # service name
            self.create_request_handler,  # cb to create the request
            [],  # outcomes. Includes SUCCEED, ABORT, TIMEOUT by default
            self.response_handler,  # cb to process the response
            timeout = 2.5 #seconds 
        )

    def create_request_handler(self, blackboard: Blackboard) -> ChangeState.Request:

        req = ChangeState.Request()
        req.transition.id=1
        req.transition.label='configure'
        # print("ConfigureEtasl")
        return req

    def response_handler(self,blackboard: Blackboard,response: ChangeState.Response) -> str:

        # print("Service response success: " + str(response.success))
        blackboard.success = response.success
        if not response.success:
            return ABORT
        
        # time.sleep(1)
        return SUCCEED

class ActivateEtasl(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            ChangeState,  # srv type
            "/etasl_node/change_state",  # service name
            self.create_request_handler,  # cb to create the request
            [],  # outcomes. Includes SUCCEED, ABORT, TIMEOUT by default
            self.response_handler,  # cb to process the response
            timeout = 2.5 #seconds 
        )

    def create_request_handler(self, blackboard: Blackboard) -> ChangeState.Request:

        req = ChangeState.Request()
        req.transition.id=1
        req.transition.label='activate'
        # print("ActivateEtasl")
        return req

    def response_handler(self,blackboard: Blackboard,response: ChangeState.Response) -> str:

        # print("Service response success: " + str(response.success))
        blackboard.success = response.success
        if not response.success:
            return ABORT
        return SUCCEED

class DeactivateEtasl(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            ChangeState,  # srv type
            "/etasl_node/change_state",  # service name
            self.create_request_handler,  # cb to create the request
            [],  # outcomes. Includes SUCCEED, ABORT, TIMEOUT by default
            self.response_handler,  # cb to process the response
            timeout = 2.5 #seconds 
        )

    def create_request_handler(self, blackboard: Blackboard) -> ChangeState.Request:

        req = ChangeState.Request()
        req.transition.id=1
        req.transition.label='deactivate'
        # print("DeactivateEtasl")
        return req

    def response_handler(self,blackboard: Blackboard,response: ChangeState.Response) -> str:

        # print("Service response success: " + str(response.success))
        blackboard.success = response.success
        if not response.success:
            return ABORT
        # time.sleep(1)
        return SUCCEED

class CleanupEtasl(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            ChangeState,  # srv type
            "/etasl_node/change_state",  # service name
            self.create_request_handler,  # cb to create the request
            [],  # outcomes. Includes SUCCEED, ABORT, TIMEOUT by default
            self.response_handler,  # cb to process the response
            timeout = 2.5 #seconds 
        )

    def create_request_handler(self, blackboard: Blackboard) -> ChangeState.Request:

        req = ChangeState.Request()
        req.transition.id=1
        req.transition.label='cleanup'
        # print("CleanupEtasl")
        return req

    def response_handler(self,blackboard: Blackboard,response: ChangeState.Response) -> str:

        # print("Service response success: " + str(response.success))
        blackboard.success = response.success
        if not response.success:
            return ABORT
        # time.sleep(1)
        return SUCCEED
    
        #     req_task = TaskSpecificationFile.Request()
        # req_task.file_path = file_name
        # repl = self.call_service('readTaskSpecificationFile', req_task)

class ReadTaskSpecificationFile(ServiceState):
    def __init__(self, file_path: str) -> None:
        super().__init__(
            TaskSpecificationFile,  # srv type
            "/etasl_node/readTaskSpecificationFile",  # service name
            self.create_request_handler,  # cb to create the request
            [],  # outcomes. Includes SUCCEED, ABORT, TIMEOUT by default
            self.response_handler,  # cb to process the response
            timeout = 2.5 #seconds 
        )
        self.file_path = file_path

    def create_request_handler(self, blackboard: Blackboard) -> TaskSpecificationFile.Request:

        req = TaskSpecificationFile.Request()
        req.file_path = self.file_path
        # print("ReadTaskSpecificationFile")
        return req

    def response_handler(self,blackboard: Blackboard,response: TaskSpecificationFile.Response) -> str:

        # print("Service response success: " + str(response.success))
        blackboard.success = response.success
        if not response.success:
            return ABORT
        # time.sleep(1)
        return SUCCEED
    
class Executing(EventState):
    def __init__(self, name: str) -> None:
        super().__init__(
                         topic_name="fsm/events",  # topic name
                         outcomes=["e_finished@etasl_node",],  # explicitly list the events that can be received through the topic. Events that are not specified are ignored
                         entry_handler = None,  # entry handler callback, called once when entering
                         monitor_handler = None,  # monitor handler callback, called several times. If omitted or set to None, the default behavior is to match the topic msg to the outcome
                         exit_handler = self.exit_handler,  # exit handler callback, called once when exiting
                         state_name = name, #If omitted or set to None, no printing in colors when entering/exiting state
                         )
    

    def exit_handler(self, blackboard: Blackboard):
        # YasminNode.get_instance().get_logger().info("exit handler called")
        return
        # time.sleep(1)

def nested_etasl_state(name: str, file_path: str, robot_path: str, display_in_viewer: bool= False):

    sm = StateMachine(outcomes=[SUCCEED, ABORT])

    sm.add_state("DEACTIVATE_ETASL", DeactivateEtasl(),
            transitions={SUCCEED: "CLEANUP_ETASL",
                        ABORT: "CLEANUP_ETASL",
                        TIMEOUT: ABORT}) #This state is just added in case that etasl is already running. If not possible (ABORT) still the task continues
    
    sm.add_state("CLEANUP_ETASL", CleanupEtasl(),
            transitions={SUCCEED: "ROBOT_SPECIFICATION",
                        ABORT: "ROBOT_SPECIFICATION",
                        TIMEOUT: ABORT}) #This state is just added in case that etasl is already running. If not possible (ABORT) still the task continues

    sm.add_state("ROBOT_SPECIFICATION", ReadTaskSpecificationFile(robot_path),
            transitions={SUCCEED: "TASK_SPECIFICATION",
                        ABORT: ABORT,
                        TIMEOUT: ABORT})

    sm.add_state("TASK_SPECIFICATION", ReadTaskSpecificationFile(file_path),
            transitions={SUCCEED: "CONFIG_ETASL",
                        ABORT: ABORT,
                        TIMEOUT: ABORT})

    sm.add_state("CONFIG_ETASL", ConfigureEtasl(),
            transitions={SUCCEED: "ACTIVATE_ETASL",
                        ABORT: ABORT,
                        TIMEOUT: ABORT})
    
    state_name = 'RUNNING_{}'.format(name)

    sm.add_state("ACTIVATE_ETASL", ActivateEtasl(),
            transitions={SUCCEED: state_name,
                        ABORT: ABORT,
                        TIMEOUT: ABORT})
    
    sm.add_state(state_name, Executing(name),
            transitions={"e_finished@etasl_node": SUCCEED,
                        ABORT: ABORT})
    
    if display_in_viewer:
        YasminViewerPub('{} (nested FSM)'.format(name), sm)

    return sm
