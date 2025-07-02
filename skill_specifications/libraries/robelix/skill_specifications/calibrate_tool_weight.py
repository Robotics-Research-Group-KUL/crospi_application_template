from utils import get_cog
import rclpy
import sys

from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *

import math

from std_srvs.srv import Empty
from geometry_msgs.msg import Wrench

import numpy as np
import pdb
import os
sm_path = os.path.dirname(os.path.abspath(__file__))
from rclpy.qos import QoSProfile
import json

from scipy.spatial.transform import Rotation as R

def run_while_publishing(sm):
    """
    small state machine that executes `sm` while publishing graphviz to a topic /gz
    """

    #do_not_expand_types doesn't work for now...
    return Concurrent("concurrent",[
            sm,
            GraphvizPublisher("publisher","/gz",sm,None,skip=10,do_not_expand_types=[eTaSL_StateMachine])
    ])

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

    def set_outcome(self, outcome):
        with self.outcome_lock:
            self.outcome = outcome

    def get_outcome(self):
        with self.outcome_lock:
            outcome = self.outcome
        return outcome
    
def state_machine_cog(node=None):
    """
    """
    wait_time = 1.0

    angle_diff = np.pi/4

    RotationM_1 = R.from_euler('ZYX', [0, 0, np.pi], degrees=False).as_matrix()
    quaternion_1 = R.from_matrix(RotationM_1).as_quat()

    RotationM_2 = R.from_euler('ZYX', [0, angle_diff, 0], degrees=False).as_matrix() @ RotationM_1
    quaternion_2 = R.from_matrix(RotationM_2).as_quat()

    RotationM_3 = R.from_euler('ZYX', [0, -angle_diff, 0], degrees=False).as_matrix() @ RotationM_1
    quaternion_3 = R.from_matrix(RotationM_3).as_quat()

    RotationM_4 = R.from_euler('ZYX', [0, 0, angle_diff], degrees=False).as_matrix() @ RotationM_1
    quaternion_4 = R.from_matrix(RotationM_4).as_quat()

    RotationM_5 = R.from_euler('ZYX', [0, 0, -angle_diff], degrees=False).as_matrix() @ RotationM_1
    quaternion_5 = R.from_matrix(RotationM_5).as_quat()

    x = 0.0
    y = 0.4
    z = 1.8

    class ReadLogWrenchFromTopic(Generator):
            def __init__(self, topic_name:str, bb_key:str, node: Node = None):
                super().__init__("ReadFromTopic",[SUCCEED])
                self.node = node
                self.last_msg = None
                self.topic = topic_name
                self.bb_key = bb_key

            def cb_msg(self,msg):
                self.last_msg = msg

            def co_execute(self,blackboard):
                # subscribe to the topic
                self.node.create_subscription(Wrench,self.topic,self.cb_msg, QoSProfile(depth=1))
                # store the message in the blackboard
                # TODO: Add a timeout
                while self.last_msg is None:
                    yield TICKING
                
                blackboard[self.bb_key]=self.last_msg
                # blackboard[self.bb_key]["force"] = {}
                # blackboard[self.bb_key]["torque"] = {}
                # blackboard[self.bb_key]["force"]["x"] = self.last_msg.force.x
                # blackboard[self.bb_key]["force"]["y"] = self.last_msg.force.y
                # blackboard[self.bb_key]["force"]["z"] = self.last_msg.force.z
                # blackboard[self.bb_key]["torque"]["x"] = self.last_msg.torque.x
                # blackboard[self.bb_key]["torque"]["y"] = self.last_msg.torque.y
                # blackboard[self.bb_key]["torque"]["z"] = self.last_msg.torque.z
                # # print("Wrench Recorded: ", blackboard[self.bb_key])
                # Destroy the subscription
                self.node.destroy_subscription(self.topic)
                yield SUCCEED

    class ComputeAndSaveCOG(Generator):
            def __init__(self, file_path:str, poses: list, wrench_keys: list):
                super().__init__("ReadFromTopic",[SUCCEED])
                self.file_path = file_path
                self.poses = poses
                self.wrench_keys = wrench_keys

            def co_execute(self,blackboard):
                wrenches = [blackboard[key] for key in self.wrench_keys]
                cog = get_cog(wrenches, self.poses)
                # Serializing json
                json_object = json.dumps(cog, indent=4)
                # Writing to file_path
                with open(self.file_path, "w") as outfile:
                    outfile.write(json_object)
                yield SUCCEED

    return  Sequence("my_sequence", children=[
                                        eTaSL_StateMachine("movinghome","MovingHome",node=node),
                                        eTaSL_StateMachine("moving_pretare","MoveCartesianPose",node=node, 
                                                                cb= lambda bb: {"desired_pose": [x, y, z, quaternion_1[0], quaternion_1[1], quaternion_1[2], quaternion_1[3]]}),
                                        TimedWait("timed_wait",Duration(seconds=wait_time),node=node ),
                                        ServiceClient("taring_ft_sensor","/schunk/tare_sensor", Empty, [SUCCEED], node=None),
                                        TimedWait("timed_wait",Duration(seconds=wait_time),node=node ),
                                        eTaSL_StateMachine("moving_first_pose","MoveCartesianPose",node=node, 
                                                                cb= lambda bb: {"desired_pose": [x, y, z, quaternion_2[0], quaternion_2[1], quaternion_2[2], quaternion_2[3]]}),
                                        TimedWait("timed_wait",Duration(seconds=wait_time),node=node ),
                                        ReadLogWrenchFromTopic("/schunk/wrench", "wrench_1", node=node),
                                        eTaSL_StateMachine("moving_second_pose","MoveCartesianPose",node=node,
                                                                cb= lambda bb: {"desired_pose": [x, y, z, quaternion_3[0], quaternion_3[1], quaternion_3[2], quaternion_3[3]]}),
                                        TimedWait("timed_wait",Duration(seconds=wait_time),node=node ),
                                        ReadLogWrenchFromTopic("/schunk/wrench", "wrench_2", node=node),
                                        eTaSL_StateMachine("moving_third_pose","MoveCartesianPose",node=node,
                                                                cb= lambda bb: {"desired_pose": [x, y, z, quaternion_4[0], quaternion_4[1], quaternion_4[2], quaternion_4[3]]}),
                                        TimedWait("timed_wait",Duration(seconds=wait_time),node=node ),
                                        ReadLogWrenchFromTopic("/schunk/wrench", "wrench_3", node=node),
                                        eTaSL_StateMachine("moving_fourth_pose","MoveCartesianPose",node=node,
                                                                cb= lambda bb: {"desired_pose": [x, y, z, quaternion_5[0], quaternion_5[1], quaternion_5[2], quaternion_5[3]]}),
                                        TimedWait("timed_wait",Duration(seconds=wait_time),node=node ),
                                        ReadLogWrenchFromTopic("/schunk/wrench", "wrench_4", node=node),
                                        ComputeAndSaveCOG("{}/testing.json".format(sm_path), poses=[RotationM_2, RotationM_3, RotationM_4, RotationM_5], 
                                                            wrench_keys=["wrench_1", "wrench_2", "wrench_3", "wrench_4"]),
                                        eTaSL_StateMachine("moving_pretare","MoveCartesianPose",node=node, 
                                                                cb= lambda bb: {"desired_pose": [x, y, z, quaternion_1[0], quaternion_1[1], quaternion_1[2], quaternion_1[3]]}),
                                        # Message("Robot is finished")
                                    ] )


def main(args=None):

    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("blade_insertion_skill")

    blackboard = {}

    load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/robelix/tasks/calibrate_tool_weight.json", blackboard)

    cog_sm = state_machine_cog(my_node)
    sm = run_while_publishing(cog_sm)

    # prints a graphviz representation of sm:
    vis = GraphViz_Visitor()
    sm.accept(vis)
    vis.print()

    sm.accept(vis)
    vis.print()

    # import pdb
    # pdb.set_trace()

    runner = BeTFSMRunner(my_node,sm,blackboard,0.01)

    rclpy.spin(my_node)

    print("Exiting BeTFSMRunner")

if __name__ == "__main__":
    sys.exit(main(sys.argv))