#!/usr/bin/env python3


import rclpy

from rclpy.node import Node
from yasmin_ros.yasmin_node import YasminNode #Necessary to use node logger and other functionalities

from yasmin import Blackboard
from yasmin import StateMachine
from yasmin import State

from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub
from rclpy.executors import MultiThreadedExecutor

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



from yasmin_action_server.cb_state_machine import cbStateMachine  # a StateMachine with configurable callbacks
from yasmin_action_server.yasmin_action_server import YasminActionServer,set_logger, action_state_cb, CancelState,EmptyStateMachine
from yasmin_viewer import YasminViewerPub

import json
import jsonschema
import time







class ResetCounterState(State):
    def __init__(self) -> None:
        super().__init__(["next"])

    def execute(self, blackboard: Blackboard) -> str:
        time.sleep(1)
        if not "counter" in blackboard:
            blackboard["counter"]=0
        blackboard["counter"] = 0
        return "next"


class CounterState(State):
    def __init__(self,maxcount) -> None:
        super().__init__(["counting", "finished"])
        self.maxcount=maxcount

    def execute(self, blackboard: Blackboard) -> str:
        time.sleep(1)
        if not "counter" in blackboard:
            blackboard["counter"]=0
        
        # just as an example: put it in the feedback:
        if not "feedback" in blackboard:
            blackboard["feedback"]={}
        blackboard["feedback"]["counter"]=blackboard["counter"]

        # determine transition
        if blackboard["counter"] < self.maxcount:
            blackboard["counter"] = blackboard["counter"]+1
            return "counting"
        else:
            return "finished"





class Configuring(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED,])

    def execute(self, blackboard: Blackboard) -> str:
        print(Style.BRIGHT + Fore.GREEN + 'ENTERING STATE CONFIGURING' + Style.RESET_ALL) #EtaslState does this automatically
        # time.sleep(1)

        task_index = etasl_utils.get_index("MovingDown",blackboard)
        blackboard["tasks"][task_index]["task_specification"]["parameters"]["maxacc"] = 2

        print(Style.BRIGHT + Fore.RED + 'EXITING STATE CONFIGURING' + Style.RESET_ALL) #EtaslState does this automatically

        return SUCCEED
    
class MyStateMachine(cbStateMachine):
    """
    A small state machine to demonstrated the above implemented nodes
    """
    def __init__(self,maxcount=5):
        super().__init__(outcomes=["success","cancel"],statecb=action_state_cb)
        # add states

      
        self.add_state("CONFIGURING", Configuring(),
                        transitions={SUCCEED: "RESET_CTR"})
        
        self.add_state(
            "RESET_CTR",
            ResetCounterState(),
            transitions={
                "next":"CTR"
            }
        )
        self.add_state(
            "CTR",
            CounterState(maxcount=maxcount),
            transitions={
                "counting": "MovingDown",
                "finished": "success"  #you can connect a transition to another transition!
            }
        )        
        self.add_state("MovingDown", etasl_utils.nested_etasl_state(name="MovingDown", display_in_viewer=True),
                        transitions={SUCCEED: "MovingUp", 
                                    ABORT: "cancel"})
        
        self.add_state("MovingUp", etasl_utils.nested_etasl_state(name="MovingUp", display_in_viewer=True),
                    transitions={SUCCEED: "CHECK_CANCEL", 
                                    ABORT: "cancel"})
        self.add_state(
            "CHECK_CANCEL",
            CancelState(),
            transitions={
                "next":"CTR" # the cancel outcome is an outcome of this state machine
            }
        ) 

        # if you'd like to validate your inputs, add this member [Optional]:        
        # self.input_parameters_schema=json.loads("""
        #         {
        #             "title": "position",
        #             "description": "origin of frame",
        #             "type": "object",
        #             "properties" : {
        #                 "x" : { "type" : "number" },
        #                 "y" : { "type" : "number" },
        #                 "z" : { "type" : "number" }
        #             },
        #             "required" : [ "x","y","z"],
        #             "additionalProperties" : false
        #         }
        # """)
   
class MovingDown(cbStateMachine):
    """
    """
    def __init__(self):
        super().__init__(outcomes=["success","cancel"],statecb=action_state_cb)
      
        self.add_state("CONFIGURING", Configuring(),
                        transitions={SUCCEED: "MovingDown"})
       
        self.add_state("MovingDown", etasl_utils.nested_etasl_state(name="MovingDown", display_in_viewer=True),
                        transitions={SUCCEED: "success", 
                                    ABORT: "cancel"})
        self.set_start_state("CONFIGURING")

class MovingUp(cbStateMachine):
    """
    """
    def __init__(self,maxcount=5):
        super().__init__(outcomes=["success","cancel"],statecb=action_state_cb)
      
        self.add_state("CONFIGURING", Configuring(),
                        transitions={SUCCEED: "MovingUp"})
               
        self.add_state("MovingUp", etasl_utils.nested_etasl_state(name="MovingUp", display_in_viewer=True),
                    transitions={SUCCEED:  "success", 
                                    ABORT: "cancel"})
        self.set_start_state("CONFIGURING")

# main
def main(args=None):

    print("yasmin_etasl")
    rclpy.init(args=args)

    blackboard = Blackboard()
    etasl_utils.load_task_list("task_configuration/nested_sequence_action.json",blackboard)

    sm = MyStateMachine(maxcount=3)
    md = MovingDown()
    mu = MovingUp()
    statemachines = {"nested_sequence":sm,"MovingDown":md,"MovingUp":mu}

    empty_statemachine = EmptyStateMachine()
    action_server = YasminActionServer(blackboard,statemachines,name="yasmin_action_server")
    pub = YasminViewerPub("error", sm,10,node=action_server)
    action_server.set_viewer(pub)
    set_logger(action_server)
    

    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)    
    executor.spin()
    
    action_server.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
