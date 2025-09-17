from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *
from .gripper_actions import MoveGripperToPosition, GripPart

class PickupFixtureSkill(Sequence):
    """
    """
    def __init__(self, node, id, skill_params):
        super().__init__("Pickup_fixture_skill")
        self.node = node
        self.skill_params = skill_params

        self.add_state(MoveGripperToPosition(finger_position = 0.0, gripping_velocity = 50.0, node=node))
        self.add_state(eTaSL_StateMachine("MovingPickupHome","MovingPickupHome",node=node))
        self.add_state(eTaSL_StateMachine("MovingToPrePickup","MovingToPrePickup",node=node))
        # self.add_state(eTaSL_StateMachine("MovingToPickup","MovingToPickup",
        #                                                 cb = lambda bb: {
        #                                                     "desired_pose": skill_params["pick_up_location"]
        #                                                 },
        #                                                 node=node))
        self.add_state(eTaSL_StateMachine("MovingToPickup","MovingToPickup",node=node))
        self.add_state(MoveGripperToPosition(finger_position = 28.0, gripping_velocity = 50.0, node=node))
        self.add_state(eTaSL_StateMachine("MovingToPrePickup","MovingToPrePickup",node=node))

        # self.add_state(GripPart(gripping_velocity=30.0, gripping_force=60.0, gripping_direction=True, node=node))
        