from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *
from .gripper_actions import MoveGripperToPosition, GripPart

import numpy as np

class PickupFixtureSkill(Sequence):
    """
    """
    def __init__(self, node, fixture_type:str, skill_params:dict):
        super().__init__("Pickup_fixture_skill")
        self.node = node
        self.fixture_type = fixture_type
        self.skill_params = skill_params
        print("Initial")
        self.add_state(MoveGripperToPosition(finger_position = 0.0, gripping_velocity = 50.0, node=node))
        print("State added 1")
        
        self.add_state(eTaSL_StateMachine("MovingPickupHome","MovingPickupHome",
                                          cb = lambda bb: {
                                                            "target_x_coordinate_mav": skill_params["fixture_x_coordinate"]
                                                        }, 
                                          node=node))
        print("State added 2")
        if fixture_type=="CHANNEL":
            self.channel_fixture_pickup()
        elif fixture_type=="PIVOT":
            self.pivot_fixture_pickup()
        else:
            raise ValueError(f"Unknown fixture type {fixture_type}")
        
        print("State added 3")


    def channel_fixture_pickup(self):
        self.add_state(ComputePrePickupPose(bb_pickup_location="channel_pickup_cartesian_pose", 
                                            bb_pre_pickup_location="channel_prepickup_cartesian_pose", pre_pickup_offset=0.22, fixture_type="CHANNEL"))
        self.add_state(eTaSL_StateMachine("MovingToPrePickupChannel_3","MovingToPrePickupChannel_3",node=self.node))
        self.add_state(eTaSL_StateMachine("MovingToPickupChannel","MovingToPickupChannel",node=self.node))
        self.add_state(MoveGripperToPosition(finger_position = 30.0, gripping_velocity = 50.0, node=self.node))
        self.add_state(eTaSL_StateMachine("MovingToPrePickupChannel_1","MovingToPrePickupChannel_1",node=self.node))
        self.add_state(eTaSL_StateMachine("MovingToPrePickupChannel_2","MovingToPrePickupChannel_2",node=self.node))
        self.add_state(eTaSL_StateMachine("MovingToPrePickupChannel_3","MovingToPrePickupChannel_3",node=self.node))

    def pivot_fixture_pickup(self):
        self.add_state(ComputePrePickupPose(bb_pickup_location="pivot_pickup_cartesian_pose", 
                                            bb_pre_pickup_location="pivot_prepickup_cartesian_pose", pre_pickup_offset=0.22, fixture_type="PIVOT"))
        self.add_state(eTaSL_StateMachine("MovingToPrePickupPivot_3","MovingToPrePickupPivot_3",node=self.node))
        self.add_state(eTaSL_StateMachine("MovingToPickupPivot","MovingToPickupPivot",node=self.node))
        self.add_state(MoveGripperToPosition(finger_position = 30.0, gripping_velocity = 50.0, node=self.node))
        self.add_state(eTaSL_StateMachine("MovingToPrePickupPivot_1","MovingToPrePickupPivot_1",node=self.node))
        self.add_state(eTaSL_StateMachine("MovingToPrePickupPivot_2","MovingToPrePickupPivot_2",node=self.node))
        self.add_state(eTaSL_StateMachine("MovingToPrePickupPivot_3","MovingToPrePickupPivot_3",node=self.node))


class ComputePrePickupPose(Generator):
    def __init__(self, bb_pickup_location:str, bb_pre_pickup_location:str,  fixture_type:str, pre_pickup_offset:float=0.2):

        super().__init__("ComputePrePickupPose",[SUCCEED])
        self.bb_location = bb_pickup_location
        self.bb_pre_pickup_location = bb_pre_pickup_location
        self.pre_pickup_offset = pre_pickup_offset
        self.fixture_type = fixture_type

    def co_execute(self,blackboard):
        if self.fixture_type=="PIVOT":
            bb_location_1 = self.bb_pre_pickup_location+"_1"
            bb_location_2 = self.bb_pre_pickup_location+"_2"
            bb_location_3 = self.bb_pre_pickup_location+"_3"

            pickup_pose = blackboard["application_params"][self.bb_location]
            pre_pickup_pose_1 = np.array(pickup_pose)
            pre_pickup_pose_1[1] -= self.pre_pickup_offset*np.sin(20.0*np.pi/180.0)/3
            pre_pickup_pose_1[2] += self.pre_pickup_offset*np.cos(20.0*np.pi/180.0)/3
            blackboard["output_param"][bb_location_1] = pre_pickup_pose_1.tolist()

            pre_pickup_pose_2 = pre_pickup_pose_1.copy()
            pre_pickup_pose_2[1] -= self.pre_pickup_offset*np.cos(20.0*np.pi/180.0)/3
            pre_pickup_pose_2[2] -= self.pre_pickup_offset*np.sin(20.0*np.pi/180.0)/3
            blackboard["output_param"][bb_location_2] = pre_pickup_pose_2.tolist()

            pre_pickup_pose_3 = pre_pickup_pose_2.copy()
            pre_pickup_pose_3[1] -= self.pre_pickup_offset*np.sin(20.0*np.pi/180.0)*2/3
            pre_pickup_pose_3[2] += self.pre_pickup_offset*np.cos(20.0*np.pi/180.0)*2/3
            blackboard["output_param"][bb_location_3] = pre_pickup_pose_3.tolist()

        elif self.fixture_type=="CHANNEL":
            bb_location_1 = self.bb_pre_pickup_location+"_1"
            bb_location_2 = self.bb_pre_pickup_location+"_2"
            bb_location_3 = self.bb_pre_pickup_location+"_3"

            pickup_pose = blackboard["application_params"][self.bb_location]
            pre_pickup_pose_1 = np.array(pickup_pose)
            pre_pickup_pose_1[1] -= self.pre_pickup_offset*np.sin(20.0*np.pi/180.0)/3
            pre_pickup_pose_1[2] += self.pre_pickup_offset*np.cos(20.0*np.pi/180.0)/3
            blackboard["output_param"][bb_location_1] = pre_pickup_pose_1.tolist()

            pre_pickup_pose_2 = pre_pickup_pose_1.copy()
            pre_pickup_pose_2[1] -= self.pre_pickup_offset*np.cos(20.0*np.pi/180.0)/3
            pre_pickup_pose_2[2] -= self.pre_pickup_offset*np.sin(20.0*np.pi/180.0)/3
            blackboard["output_param"][bb_location_2] = pre_pickup_pose_2.tolist()

            pre_pickup_pose_3 = pre_pickup_pose_2.copy()
            pre_pickup_pose_3[1] -= self.pre_pickup_offset*np.sin(20.0*np.pi/180.0)*2/3
            pre_pickup_pose_3[2] += self.pre_pickup_offset*np.cos(20.0*np.pi/180.0)*2/3
            blackboard["output_param"][bb_location_3] = pre_pickup_pose_3.tolist()

        yield SUCCEED