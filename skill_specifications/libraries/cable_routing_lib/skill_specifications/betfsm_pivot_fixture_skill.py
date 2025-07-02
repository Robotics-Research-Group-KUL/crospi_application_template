from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *
from .gripper_actions import MoveGripperToPosition, GripPart

from rclpy.node import Node

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


class PivotFixtureSkill(Sequence):
    """
    """
    def __init__(self, node, id, skill_params):
        """
        Constructor for the PivotFixtureSkill class.
        :param node: The node instance.
        :param id: The ID of the skill.
        :param skill_params: A dictionary containing the parameters for the skill.

        skill_params is a dictionary with the following keys:
        - "turning_dir_sliding" int: Direction of sliding (0 or 1).
        - "desired_pos" [x,y,z]: Desired pose for sliding.
        - "gripper_vel" int (0-100): Gripper velocity.
        - "gripper_force" int (0-100): Gripper force.
        - "gripper_direction" bool: Gripper direction (True or False).
        - "z_down" int: Z-axis down position.
        - "frame_next_fixture" [x,y,z,i,j,k,w]: Frame of the next fixture.
        - "turning_dir_pivoting" int: Direction of pivoting (0 or 1).
        """
        super().__init__("Pivot_fixture_skill")
        self.node = node
        self.skill_params = skill_params

        self.add_state(MoveGripperToPosition(finger_position=skill_params["cable_slide_pos"], gripping_velocity = skill_params["gripper_vel"], node=node))
        self.add_state(eTaSL_StateMachine("cableSliding","CableSliding",
                                                        cb = lambda bb: {
                                                            "turning_dir": skill_params["turning_dir_sliding"],
                                                            "desired_pos": skill_params["desired_pos"]
                                                        },
                                                        node=node))
        self.add_state(GripPart(gripping_velocity=skill_params["gripper_vel"], gripping_force=skill_params["gripper_force"], 
                                gripping_direction=skill_params["gripper_direction"], node=node))
        self.add_state(eTaSL_StateMachine("pivotAligning","PivotAligning",
                                                        cb = lambda bb: {
                                                            "previous_to_current_fixture": skill_params["previous_to_current_fixture"],
                                                            "z_down": skill_params["z_down"]
                                                        },
                                                        node=node))
        self.add_state(eTaSL_StateMachine("cablePivoting","CablePivoting",
                                                        cb = lambda bb: {
                                                            "frame_next_fixture_wrt_board": skill_params["frame_next_fixture_wrt_board"],
                                                            "turning_dir_pivoting": skill_params["turning_dir_pivoting"],
                                                        },
                                                        node=node))
        self.add_state(MoveGripperToPosition(finger_position = skill_params["cable_slide_pos"], gripping_velocity = skill_params["gripper_vel"], node=node))


def test_gripper(node=None):
    """
    """
    return  Sequence("gripper_test", children=[
                                        MoveGripperToPosition(finger_position=0.0, gripping_velocity=50.0, node=node),
                                        GripPart(gripping_velocity=15.0, gripping_force=80.0, gripping_direction=True, node=node),
                                        MoveGripperToPosition(finger_position=20.0, gripping_velocity=50.0, node=node),
                                        MoveGripperToPosition(finger_position=0.0, gripping_velocity=50.0, node=node),
                                        MoveGripperToPosition(finger_position=20.0, gripping_velocity=50.0, node=node)
                                    ] )

def main(args=None):
    """
    """
    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("pcb_insertion_skill")

    set_logger("default",my_node.get_logger())

    sm_gripper_test = test_gripper(node=my_node)

    blackboard = {}

    runner = BeTFSMRunner(my_node,sm_gripper_test,blackboard,0.01)

    rclpy.spin(my_node)


if __name__ == "__main__":
    import sys
    import rclpy
    sys.exit(main(sys.argv))