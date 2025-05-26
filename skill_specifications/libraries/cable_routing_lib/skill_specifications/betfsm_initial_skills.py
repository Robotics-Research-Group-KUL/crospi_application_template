from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *
from gripper_actions import MoveGripperToPosition, GripPart
import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

class TFListenerState(Generator):
    def __init__(self, name: str, parent_frame: str, child_frame: str, timeout: Duration = Duration(seconds=1.0), node: Node = None):
        outcomes = [SUCCEED, TIMEOUT, TICKING]
        super().__init__(name, outcomes)

        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.timeout = timeout
        self.name = name

        self.node = node or self._get_node()
        self.clock = self.node.get_clock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def _get_node(self):
        return BeTFSMNode.get_instance()

    def co_execute(self, blackboard):
        # get_logger("tf").info(f"Looking up TF from '{self.parent_frame}' to '{self.child_frame}'")
        start_time = self.clock.now()

        while rclpy.ok():
            try:
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    self.parent_frame,
                    self.child_frame,
                    rclpy.time.Time()
                )
                # Store transform in blackboard
                transformation_name = f"{self.parent_frame}_to_{self.child_frame}"
                blackboard["output_param"][self.name][transformation_name] = [transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w]
                # get_logger("tf").info(f"Transform received and stored on blackboard")
                yield SUCCEED

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                if self.timeout != Duration() and self.clock.now() - start_time > self.timeout:
                    # get_logger("tf").warn(f"TF lookup from '{self.parent_frame}' to '{self.child_frame}' timed out")
                    yield TIMEOUT
                else:
                    yield TICKING


class GoToCableStart(Sequence):
    """
    """
    def __init__(self, node, id=0):
        """
        Constructor for the ChannelFixtureSkill class.
        :param node: The node instance.
        :param id: The ID of the skill.
        :param skill_params: A dictionary containing the parameters for the skill.

        skill_params is a dictionary with the following keys:
        - "turning_dir_sliding" int: Direction of sliding (0 or 1).
        - "desired_pos" [x,y,z]: Desired pose for sliding.
        - "gripper_vel" int (0-100): Gripper velocity.
        - "gripper_force" int (0-100): Gripper force.
        - "gripper_direction" bool: Gripper direction (True or False).
        - "channel_aligning_pose" [x,y,z,i,j,k,w]: Pose for channel aligning.
        - "channel_inserting_pose" [x,y,z]: Pose for channel inserting.
        - "cable_slide_pos" int: Position of the cable slide.
        """
        super().__init__("Channel_fixture_skill")
        self.node = node

        self.add_state(TFListenerState("tf_listener", "base_link", "board", Duration(seconds=5), node=node))
        self.add_state(eTasl_StateMachine("goingToCableStart", "GoingToCableStart", node=node))
        