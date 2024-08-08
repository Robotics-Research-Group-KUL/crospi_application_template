#!/usr/bin/env python3


import rclpy

from rclpy.node import Node
from yasmin_ros.yasmin_node import YasminNode

from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin import State

from yasmin_ros import ServiceState
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# from lifecycle_msgs.srv import ChangeState_Response

from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

from etasl_interfaces.srv import TaskSpecificationFile
from etasl_interfaces.srv import TaskSpecificationString

from functools import partial
from colorama import Fore, Back, Style


import etasl_yasmin_utils as etasl_utils
from event_state import EventState

import time

import pdb

class Configured(State):
    def __init__(self, serv_manager) -> None:
        super().__init__([SUCCEED,])
        self.serv_manager = serv_manager

    def execute(self, blackboard: Blackboard) -> str:
        print(Style.BRIGHT + Fore.GREEN + 'ENTERING STATE CONFIGURED' + Style.RESET_ALL) #EventState does this automatically
        self.serv_manager.deactivate()
        self.serv_manager.cleanup()
        #
        #
        # Here is where we will add the etasl ports and link them with topics of other components
        #
        #
        # time.sleep(1)
        print(Style.BRIGHT + Fore.RED + 'EXITING STATE CONFIGURED' + Style.RESET_ALL) #EventState does this automatically

        return SUCCEED
    
class MovingCartesian(EventState):
    def __init__(self, serv_manager) -> None:
        super().__init__(
                         topic_name="fsm/events",  # topic name
                         outcomes=["e_finished@etasl_node",],  # explicitly list the events that can be received through the topic. Events that are not specified are ignored
                         entry_handler = self.entry_handler,  # entry handler callback, called once when entering
                         monitor_handler = None,  # monitor handler callback, called several times. If omitted or set to None, the default behavior is to match the topic msg to the outcome
                         exit_handler = self.exit_handler,  # exit handler callback, called once when exiting
                         state_name = "MovingCartesian", #If omitted or set to None, no printing in colors when entering/exiting state
                         )
        self.serv_manager = serv_manager
    
    def entry_handler(self, blackboard: Blackboard):
        self.serv_manager.deactivate()
        self.serv_manager.cleanup()
        self.serv_manager.readTaskSpecificationFile(blackboard=blackboard,file_name= "move_cartesianspace.lua", rel_shared_dir=True)
        self.serv_manager.configure()
        self.serv_manager.activate()
        return

    def exit_handler(self, blackboard: Blackboard):
        YasminNode.get_instance().get_logger().info("exit handler called")
        return
        # time.sleep(1)
    

class MovingHome(EventState):
    def __init__(self, serv_manager) -> None:
        super().__init__(
                         topic_name="fsm/events",  # topic name
                         outcomes=["e_finished@etasl_node",],  # explicitly list the events that can be received through the topic. Events that are not specified are ignored
                         entry_handler = self.entry_handler,  # entry handler callback, called once when entering
                         monitor_handler = None,  # monitor handler callback, called several times. If omitted or set to None, the default behavior is to match the topic msg to the outcome
                         exit_handler = self.exit_handler,  # exit handler callback, called once when exiting
                         state_name = "MovingHome", #If omitted or set to None, no printing in colors when entering/exiting state
                         )
        self.serv_manager = serv_manager
    
    def entry_handler(self, blackboard: Blackboard):
        self.serv_manager.deactivate()
        self.serv_manager.cleanup()
        self.serv_manager.readTaskSpecificationFile(blackboard=blackboard,file_name= "move_jointspace_trap.lua", rel_shared_dir=True)
        print("read task spacecification... waiting...")
        # time.sleep(5)
        self.serv_manager.configure()
        print("configured... waiting...")
        # time.sleep(5)
        self.serv_manager.activate()
        print("activated... waiting...")
        # time.sleep(5)

        return

    def exit_handler(self, blackboard: Blackboard):
        YasminNode.get_instance().get_logger().info("exit handler called")
        return
        # time.sleep(1)

class MovingJoystick(EventState):
    def __init__(self, serv_manager) -> None:
        super().__init__(
                         topic_name="fsm/events",  # topic name
                         outcomes=["e_finished@etasl_node",],  # explicitly list the events that can be received through the topic. Events that are not specified are ignored
                         entry_handler = self.entry_handler,  # entry handler callback, called once when entering
                         monitor_handler = None,  # monitor handler callback, called several times. If omitted or set to None, the default behavior is to match the topic msg to the outcome
                         exit_handler = self.exit_handler,  # exit handler callback, called once when exiting
                         state_name = "MovingJoystick", #If omitted or set to None, no printing in colors when entering/exiting state
                         )
        self.serv_manager = serv_manager
    
    def entry_handler(self, blackboard: Blackboard):
        self.serv_manager.deactivate()
        self.serv_manager.cleanup()
        self.serv_manager.readTaskSpecificationFile(blackboard=blackboard, file_name= "ramasa.lua", rel_shared_dir=True)
        print("read task spacecification... waiting...")
        # time.sleep(5)
        self.serv_manager.configure()
        print("configured... waiting...")
        # time.sleep(5)
        self.serv_manager.activate()
        print("activated... waiting...")
        # time.sleep(5)
        return

    def exit_handler(self, blackboard: Blackboard):
        YasminNode.get_instance().get_logger().info("exit handler called")
        return
        # time.sleep(1)


def test_callback(blackboard: Blackboard) -> str:
    blackboard.a = 10
    blackboard.b = 5
    time.sleep(2)
    return SUCCEED


# main
def main(args=None):

    print("yasmin_etasl")
    rclpy.init(args=args)

    # task_spec_cb = partial(etasl.readTaskSpecificationFile,file_name= "move_cartesianspace.lua", rel_shared_dir=True)
    
    sm_example = StateMachine(outcomes=["finished_outer"])

    serv_manager = etasl_utils.ServiceManager()
    serv_manager.define_services()#Creates all the etasl service clients and adds them to self.etasl_clients 

    
    sm_example.add_state("CONFIGURED", Configured(serv_manager),
                    transitions={SUCCEED: "MovingHome"})

    sm_example.add_state("MovingHome", MovingHome(serv_manager),
                    transitions={"e_finished@etasl_node": "MovingCartesian"})
    
    sm_example.add_state("MovingCartesian", MovingCartesian(serv_manager),
                    transitions={"e_finished@etasl_node": "MovingJoystick"})
    
    sm_example.add_state("MovingJoystick", MovingJoystick(serv_manager),
                    transitions={"e_finished@etasl_node": "STATE_OUTER_B"})
    
    sm_example.add_state("STATE_OUTER_B", CbState([SUCCEED], test_callback),
                    transitions={SUCCEED: "MovingCartesian",
                            ABORT: "finished_outer"})


    # pub
    # YasminViewerPub("Lifecycle FSM", sm)
    YasminViewerPub("Complete FSM", sm_example)


    # etasl.define_services(self) #Creates all the etasl service clients and adds them to self.etasl_clients 


    # execute
    outcome = sm_example() #This function will block until the output of sm_example is achieved
    print(outcome)

    # node = EtaslFSMNode()
    # node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
