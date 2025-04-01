from rclpy.executors import MultiThreadedExecutor
from betfsm.betfsm_etasl import *
# from betfsm.graphviz_visitor import *
from betfsm.logger import get_logger,set_logger
from betfsm.graphviz_visitor import *
from betfsm.betfsm_action_server import BeTFSMActionServer,CheckForCanceledAction,WhileNotCanceled
import rclpy 

def run_while_publishing( sm):
    """
    small state machine that executes `sm` while publishing graphviz to a topic /gz
    """

    #do_not_expand_types doesn't work for now...
    return Concurrent("concurrent",[
            sm,
        GraphvizPublisher("publisher","/gz",sm,None,skip=10,do_not_expand_types=[eTaSL_StateMachine])
        #GraphvizPublisher("publisher","/gz",sm,None,skip=10,do_not_expand_types=[])
])

# define your shutdown procefdure:
class CheckingCancelAndShutdown(TickingStateMachine):
    def __init__(self,name:str,state:TickingState,srv_name:str="/etasl_node",timeout:Duration = Duration(seconds=1.0), node : Node = None):
        # execute in sequence but don't care about ABORT, only way to fail is TIMEOUT
        super().__init__(name,[CANCEL,SUCCEED])

        self.add_state(state=WhileNotCanceled("while_not_canceled",state), transitions={CANCEL:"DEACTIVATE_ETASL",SUCCEED:SUCCEED, TIMEOUT:"DEACTIVATE_ETASL"})

        self.add_state(state=LifeCycle("DEACTIVATE_ETASL",srv_name,Transition.DEACTIVATE,timeout,node),
                       transitions={SUCCEED: "CLEANUP_ETASL",  ABORT: "CLEANUP_ETASL", TIMEOUT:"CLEANUP_ETASL"} )
        self.add_state( state=LifeCycle("CLEANUP_ETASL",srv_name,Transition.CLEANUP,timeout,node),
                       transitions={SUCCEED: CANCEL, ABORT: CANCEL,TIMEOUT: CANCEL} )

def main(args=None):

    rclpy.init(args=args)
    node = BeTFSMNode.get_instance("betfsm_action_server")

    blackboard = {} 
    # load your tasks
    load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/pcb_insertion_lib/tasks/action_server_skills.json",blackboard)

    statemachines={}
    statemachines["MoveJointSpace"] = run_while_publishing(CheckingCancelAndShutdown("check", 
                                                                                      eTaSL_StateMachine("MovingJointSpace", "MovingJointSpace", node=node), 
                                                                                      node=node))
    statemachines["MoveCartesianSpace"] = run_while_publishing(CheckingCancelAndShutdown("check",
                                                                                      eTaSL_StateMachine("MovingCartesian", "MovingCartesian", node=node),
                                                                                      node=node))
    statemachines["PickupFromTray"] = run_while_publishing(CheckingCancelAndShutdown("check",
                                                                                      eTaSL_StateMachine("MovingCartesian", "MovingCartesian", node=node),
                                                                                      node=node))
    
    # TODO: Add json schema for input parameters
    # statemachines["up_and_down"].input_parameters_schema=examples.my_schema

    action_server = BeTFSMActionServer(blackboard, statemachines, 100, node, action_name="/ai_prism/move_arm")

    # single or multi threaded executor does not matter here, only default callback group is used (which is mutually exclusive)
    executor = MultiThreadedExecutor()  
    executor.add_node(action_server.node)    
    executor.spin()
    
    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()