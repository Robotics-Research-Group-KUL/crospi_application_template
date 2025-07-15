from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *
from .gripper_actions import MoveGripperToPosition, GripPart

class ChannelFixtureSkill(Sequence):
    """
    """
    def __init__(self, node, id, skill_params):
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
        self.skill_params = skill_params
        # print(skill_params)
        self.add_state(MoveGripperToPosition(finger_position = skill_params["cable_slide_pos"], gripping_velocity = skill_params["gripper_vel"], node=node))

        self.add_state(eTaSL_StateMachine("cableSliding","CableSliding",
                                                        cb = lambda bb: {
                                                            "turning_dir": skill_params["turning_dir_sliding"],
                                                            "desired_pos": skill_params["desired_pos"]
                                                        },
                                                        node=node))
        self.add_state(GripPart(gripping_velocity=skill_params["gripper_vel"], gripping_force=skill_params["gripper_force"],
                                gripping_direction=skill_params["gripper_direction"], node=node))
        self.add_state(eTaSL_StateMachine("cableChannelAligning","CableChannelAligning",
                                                        cb = lambda bb: {
                                                            "desired_pose": skill_params["channel_aligning_pose"]
                                                        },
                                                        node=node))
        self.add_state(TimedWait("TransitionWait1", Duration(seconds=0.5), node=node))
        self.add_state(eTaSL_StateMachine("cableChannelInserting","CableChannelInserting",
                                                        cb = lambda bb: {
                                                            "desired_pose": skill_params["channel_inserting_pose"],
                                                        },
                                                        node=node))
        self.add_state(TimedWait("TransitionWait2", Duration(seconds=0.5), node=node))
        self.add_state(eTaSL_StateMachine("cableTensioning","CableTensioning",
                                                        node=node))
        self.add_state(MoveGripperToPosition(finger_position = skill_params["cable_slide_pos"], gripping_velocity = skill_params["gripper_vel"], node=node))
