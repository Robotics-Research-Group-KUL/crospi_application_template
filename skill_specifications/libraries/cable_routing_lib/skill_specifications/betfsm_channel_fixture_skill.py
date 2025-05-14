from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *
from gripper_actions import MoveGripperToPosition, GripPart

class ChannelFixtureSkill(Sequence):
    """
    """
    def __init__(self, node, id, params):
        """
        Constructor for the ChannelFixtureSkill class.
        :param node: The node instance.
        :param id: The ID of the skill.
        :param params: A dictionary containing the parameters for the skill.

        params is a dictionary with the following keys:
        - "turning_dir_sliding" int: Direction of sliding (0 or 1).
        - "desired_pose" [x,y,z]: Desired pose for sliding.
        - "gripper_vel" int (0-100): Gripper velocity.
        - "gripper_force" int (0-100): Gripper force.
        - "gripper_direction" bool: Gripper direction (True or False).
        - "channel_aligning_pose" [x,y,z,i,j,k,w]: Pose for channel aligning.
        - "channel_inserting_pose" [x,y,z]: Pose for channel inserting.
        - "cable_slide_pos" int: Position of the cable slide.
        """
        super().__init__("Channel_fixture_skill")
        self.node = node
        self.params = params

        self.add_state(eTaSL_StateMachine("cableSliding","CableSliding",
                                                        lambda bb: {
                                                            "turning_dir": params["turning_dir_sliding"],
                                                            "desired_pose": params["desired_pose"]
                                                        },
                                                        node=node))
        self.add_state(GripPart(gripping_velocity=params["gripper_vel"], gripping_force=params["gripper_force"],
                                gripping_direction=params["gripper_direction"], node=node))
        self.add_state(eTaSL_StateMachine("cableChannelAligning","CableChannelAligning",
                                                        lambda bb: {
                                                            "channel_aligning_pose": params["channel_aligning_pose"]
                                                        },
                                                        node=node))
        self.add_state(eTaSL_StateMachine("cableChannelInserting","CableChannelInserting",
                                                        lambda bb: {
                                                            "channel_inserting_pose": params["channel_inserting_pose"],
                                                        },
                                                        node=node))
        self.add_state(eTaSL_StateMachine("cableTensioning","CableTensioning",
                                                        node=node))
        self.add_state(MoveGripperToPosition(finger_position = params["cable_slide_pos"], gripping_velocity = params["gripper_vel"], node=node))
