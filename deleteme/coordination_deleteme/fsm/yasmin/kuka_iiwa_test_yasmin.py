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
# from etasl_state import EtaslState


class Configuring(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED,])

    def execute(self, blackboard: Blackboard) -> str:
        print(Style.BRIGHT + Fore.GREEN + 'ENTERING STATE CONFIGURING' + Style.RESET_ALL) #EtaslState does this automatically

        print(Style.BRIGHT + Fore.RED + 'EXITING STATE CONFIGURING' + Style.RESET_ALL) #EtaslState does this automatically

        return SUCCEED
    

# main
def main(args=None):

    print("yasmin_etasl")
    rclpy.init(args=args)

    # task_spec_cb = partial(etasl.readTaskSpecificationFile,file_name= "move_cartesianspace.lua", rel_shared_dir=True)
    
    sm_out = StateMachine(outcomes=["finished_outer", ABORT])
    YasminViewerPub("Complete FSM", sm_out)


    sm_out.add_state("CONFIGURING", Configuring(),
                    transitions={SUCCEED: "MovingHome"})

    # sm_out.add_state("MovingHome", etasl_utils.nested_etasl_state(name="MovingHome", file_path="kuka_iiwa_test_etasl.lua", rel_shared_dir=True, display_in_viewer=True),
    #                 transitions={SUCCEED: "finished_outer", 
    #                              ABORT: ABORT})
    
    sm_out.add_state("MovingHome", etasl_utils.nested_etasl_state(name="MovingHome", file_path="kuka_iiwa_jointspace_trap.lua", rel_shared_dir=True, display_in_viewer=True),
                transitions={SUCCEED: "finished_outer", 
                                ABORT: ABORT})
    


    # execute
    outcome = sm_out() #This function will block until the output of sm_out is achieved
    print(outcome)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
