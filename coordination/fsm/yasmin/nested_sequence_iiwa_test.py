#!/usr/bin/env python3


import rclpy

from rclpy.node import Node
from yasmin_ros.yasmin_node import YasminNode #Necessary to use node logger and other functionalities

from yasmin import Blackboard
from yasmin import StateMachine
from yasmin import State

from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub

# from yasmin import CbState
# from yasmin_ros import ServiceState
# from yasmin_ros import MonitorState
# from lifecycle_msgs.srv import ChangeState
# from lifecycle_msgs.msg import Transition
# from lifecycle_msgs.srv import ChangeState_Response
# from std_msgs.msg import String

# from rclpy.qos import qos_profile_sensor_data

# import time
# import pdb

# from functools import partial
from colorama import Fore, Back, Style


import etasl_yasmin_utils as etasl_utils
import json
# import requests
from jsonschema import validate, ValidationError

# task_specifications_dir = "$[etasl_ros2_application_template]/etasl/task_specifications"
# robot_specifications_dir = "$[etasl_ros2_application_template]/etasl/robot_specifications"
# robot_specification = robot_specifications_dir + "/ur10.etasl.lua"



class Configuring(State):
    def __init__(self, serv_manager) -> None:
        super().__init__([SUCCEED,])
        self.serv_manager = serv_manager

    def execute(self, blackboard: Blackboard) -> str:
        print(Style.BRIGHT + Fore.GREEN + 'ENTERING STATE CONFIGURING' + Style.RESET_ALL) #EtaslState does this automatically
        # self.serv_manager.deactivate()
        # self.serv_manager.cleanup()
        # time.sleep(1)

        task_index = etasl_utils.get_index("MovingDown",blackboard)
        blackboard["tasks"][task_index]["parameters"]["maxacc"] = 2

        print(Style.BRIGHT + Fore.RED + 'EXITING STATE CONFIGURING' + Style.RESET_ALL) #EtaslState does this automatically

        return SUCCEED
    

# main
def main(args=None):

    print("yasmin_etasl")
    rclpy.init(args=args)

    blackboard = Blackboard()
    etasl_utils.load_parameters("task_configuration/nested_sequence_iiwa_test.json",blackboard)



    # task_spec_cb = partial(etasl.readTaskSpecificationFile,file_name= "move_cartesianspace.lua")
    
    sm_out = StateMachine(outcomes=["finish", ABORT])
    YasminViewerPub("Complete FSM", sm_out)


    serv_manager = etasl_utils.ServiceManager()
    serv_manager.define_services() #Creates all the etasl service clients and adds them to self.etasl_clients 

    
    sm_out.add_state("CONFIGURING", Configuring(serv_manager),
                    transitions={SUCCEED: "MovingHome"})
    
    sm_out.add_state("MovingHome", etasl_utils.nested_etasl_state(name="MovingHome", blackboard=blackboard, display_in_viewer=True),
                transitions={SUCCEED: "finish", 
                                ABORT: ABORT})
    

    # sm_out.add_state("MovingDown", etasl_utils.nested_etasl_state(name="MovingDown", blackboard=blackboard, display_in_viewer=True),
    #                 transitions={SUCCEED: "MovingUp", 
    #                              ABORT: ABORT})
    
    # sm_out.add_state("MovingUp", etasl_utils.nested_etasl_state(name="MovingUp", blackboard=blackboard, display_in_viewer=True),
    #             transitions={SUCCEED: "MovingDown", 
    #                              ABORT: ABORT})
    
    # sm_out.add_state("MovingCartesian", etasl_utils.nested_etasl_state(name="MovingCartesian", blackboard=blackboard, display_in_viewer=True),
    #             transitions={SUCCEED: "MovingJoystick", 
    #                              ABORT: ABORT})
    
    # sm_out.add_state("MovingJoystick", etasl_utils.nested_etasl_state(name="MovingJoystick", blackboard=blackboard, display_in_viewer=True),
    #         transitions={SUCCEED: "finished_outer", 
    #                              ABORT: ABORT})


    # execute
    # param = param_manager("task_configuration/nested_sequence_example_etasl.json")
    # print(blackboard)


    outcome = sm_out(blackboard) #This function will block until the output of sm_out is achieved
    print(outcome)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
