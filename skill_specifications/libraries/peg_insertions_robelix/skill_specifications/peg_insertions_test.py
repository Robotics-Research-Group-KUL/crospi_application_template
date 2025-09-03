#!/usr/bin/env python3

import rclpy
import sys

from threading import Lock
from rclpy.node import Node
from std_srvs.srv import Empty

import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R


# from betfsm.betfsm import TickingState,TICKING,Blackboard, Sequence, Message, TickingStateMachine
from betfsm.betfsm import *
from betfsm.betfsm_node import BeTFSMNode
from betfsm.betfsm_etasl import load_task_list, eTaSL_StateMachine
from betfsm.graphviz_visitor import GraphViz_Visitor
from betfsm.logger import get_logger,set_logger
from betfsm.betfsm_ros import *

from rclpy.duration import Duration

from betfsm.betfsm_action_server import CheckForCanceledAction


class ReadLogFromTopic(Generator):
    def __init__(self, topic_name:str, bb_key:str, number_data:int, node: Node = None):
        super().__init__("ReadFromTopic",[SUCCEED])
        self.node = node
        self.average_msg = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.topic = topic_name
        self.bb_key = bb_key
        self.number_data = number_data
        self.actual_data = 0

    def cb_msg(self,msg):
        euler = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]).as_euler('xyz', degrees=False)

        self.average_msg += np.array([msg.position.x, msg.position.y, msg.position.z, euler[0], euler[1], euler[2]])

        self.actual_data += 1

    def co_execute(self,blackboard):
        # subscribe to the topic
        self.node.create_subscription(Pose, self.topic, self.cb_msg, 1)
        # store the message in the blackboard
        # TODO: Add a timeout
        while self.actual_data < self.number_data:
            yield TICKING
        
        self.average_message = self.average_msg / self.actual_data
        T_matrix = np.eye(4)
        T_matrix[:3, 3] = self.average_message[:3]
        T_matrix[:3, :3] = R.from_euler('xyz', self.average_message[3:6], degrees=False).as_matrix()
        blackboard[self.bb_key] = T_matrix

        quat = R.from_euler('xyz', self.average_message[3:6], degrees=False).as_quat()
        T_root_board = [self.average_message[0], self.average_message[1], self.average_message[2],
                        quat[0], quat[1], quat[2], quat[3]]
        blackboard["output_param"]["vision_detection"] = {}
        blackboard["output_param"]["vision_detection"][self.bb_key] = T_root_board
        get_logger().info(f"Read {self.actual_data} messages from topic {self.topic}, averaged to: ------------- \n {blackboard[self.bb_key]}")

        # Destroy the subscription
        self.node.destroy_subscription(self.topic)
        yield SUCCEED



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

# main
def main(args=None):

    print("betfsm")
    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("admittance_test")

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    #set_logger("state",my_node.get_logger())

    blackboard = {}

    load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/peg_insertions_robelix/tasks/peg_insertions_test.json",blackboard)


    # TODO: Check home positions
    sm  = Sequence("CalibrationRoutine", children=[
                                                    # PrintEstimates(),
                                                    eTaSL_StateMachine("MovingHome","MovingHome", node=None),
                                                    # read the board position from the black board
                                                    ReadLogFromTopic('/charuco_detector/pose', 'initial_board_pose', 10, node=my_node),
                                                    eTaSL_StateMachine("GoingOnTopFixture","GoingOnTopFixture", node=None),
                                                    ServiceClient("taring_ft_sensor","/schunk/tare_sensor", Empty, [SUCCEED], node=None),
                                                    eTaSL_StateMachine("MakeBoardContact","MakeBoardContact", node=None),
                                                    # eTaSL_StateMachine("MoveBack","MoveBack", node=None),
                                                    # eTaSL_StateMachine("MovingHome","MovingHome", node=None),
    ])




    # # TODO: Check home positions
    # sm  = Sequence("CalibrationRoutine", children=[
    #                                                 # PrintEstimates(),
    #                                                 eTaSL_StateMachine("MovingHome","MovingHome", node=None),
    #                                                 ServiceClient("taring_ft_sensor","/schunk/tare_sensor", Empty, [SUCCEED], node=None),
    #                                                 TimedWait("timed_wait",Duration(seconds=1.0),node=None ),
    #                                                 eTaSL_StateMachine("TestAdmittance","TestAdmittance", node=None),

    #                                                 # eTaSL_StateMachine("MoveForward","MoveForward", node=None),
    #                                                 # eTaSL_StateMachine("MoveBack","MoveBack", node=None),
    #                                                 # eTaSL_StateMachine("MoveForward","MoveForward", node=None),
    #                                                 # eTaSL_StateMachine("MoveBack","MoveBack", node=None),                                                    
    #                                                 # eTaSL_StateMachine("MoveForward","MoveForward", node=None),
    #                                                 # eTaSL_StateMachine("MoveBack","MoveBack", node=None),                                                   
    #                                                 # eTaSL_StateMachine("MoveForward","MoveForward", node=None),
    #                                                 # eTaSL_StateMachine("MoveBack","MoveBack", node=None),                                                    
    #                                                 # eTaSL_StateMachine("MoveForward","MoveForward", node=None),
    #                                                 # eTaSL_StateMachine("MoveBack","MoveBack", node=None),
    #                                                 # ServiceClient("taring_ft_sensor","/schunk/tare_sensor", Empty, [SUCCEED], node=None),
    #                                                 # eTaSL_StateMachine("RotatingCheck","RotatingCheck", node=None),
    #                                                 # eTaSL_StateMachine("MovingAdmittance","MovingAdmittance", node=None)
    # ])

    # prints a graphviz representation of sm:
    vis = GraphViz_Visitor()
    sm.accept(vis)
    vis.print()

    runner = BeTFSMRunner(my_node,sm,blackboard,0.01)

    rclpy.spin(my_node)
    
    # try:
    #     while (runner.get_outcome()==TICKING):
    #         rclpy.spin_once(my_node)
    #     rclpy.shutdown()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     print("final outcome : ",runner.get_outcome())
        
    print("shutdown")

if __name__ == "__main__":
    sys.exit(main(sys.argv))
