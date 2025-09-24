''' Start on top of the fixture 
ends with the peg inserted and released '''

from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *
from .gripper_actions import MoveGripperToPosition

class PegInsertionSkill(Sequence):
    """
    """
    def __init__(self, node, id, skill_params):
        """
        """
        super().__init__("Peg_insertion_skill")
        self.node = node
        self.skill_params = skill_params

        
        self.add_state(eTaSL_StateMachine("MakeBoardContact","MakeBoardContact", node=node))
        self.add_state(eTaSL_StateMachine("SearchHole","SearchHole", node=node))
        self.add_state(MoveGripperToPosition(finger_position=25.0, gripping_velocity=50.0, node=node))
        self.add_state(eTaSL_StateMachine("InsertAfterSearch","InsertAfterSearch", node=node))
        self.add_state(MoveGripperToPosition(finger_position=30.0, gripping_velocity=50.0, node=node))
        self.add_state(eTaSL_StateMachine(f"Rotate90z{id}","Rotate90z", output_topic="/etasl/tcp_position", node=node))
       
        # self.add_state(eTaSL_StateMachine(f"Rotate90z_simulation{id}","Rotate90z_simulation", output_topic="/etasl/tcp_position", node=node))
        
        self.add_state(MoveGripperToPosition(finger_position=0.0, gripping_velocity=50.0, node=node))
        self.add_state(eTaSL_StateMachine("RetractAfterInsertion","RetractAfterInsertion", node=node))


        # self.add_state("search for hole")
        # self.add_state("align and insert with hole")
        # self.add_state("rotate to lock")
        # self.add_state("release peg")
    
