from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from rclpy.node import Node

class LCSM_MAV(Generator):
    def __init__(self, mav_node:str, transition_id:int, node:Node = None, timeout:float=5.0):

        super().__init__("ComputePrePickupPose",[SUCCEED, ABORT])
        self.mav_node = mav_node

        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node

        self.cli_change_state = node.create_client(ChangeState, f'/{mav_node}/change_state')
        self.cli_get_state = node.create_client(GetState, f'/{mav_node}/get_state')
        self.transition_id = transition_id

    def change_state_async(self, transition_id):
        """Return a future for the change_state call"""
        req = ChangeState.Request()
        req.transition.id = transition_id
        req.label = "activate"
        return self.cli_change_state.call_async(req)

    def get_state_async(self):
        """Return a future for the get_state call"""
        req = GetState.Request()
        return self.cli_get_state.call_async(req)

    def co_execute(self,blackboard):
        future = self.change_state_async(transition_id=self.transition_id)
        while not future.done():
            print("waiting for change_state...")
            yield TICKING
        response = future.result()
        if not response.success:
            self.get_logger().error(f"Failed to change state of {self.mav_node}")
            yield ABORT
        self.get_logger().info(f"Changed state of {self.mav_node}")
        # future = self.get_state_async()
        # while not future.done():
        #     yield TICKING
        # response = future.result()
        # self.get_logger().info(f"Current state of {self.mav_node} is {response.current_state.label}")
        yield SUCCEED
