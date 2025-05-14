from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *
from gripper_actions import MoveGripperToPosition, GripPart

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
    def __init__(self, node, id, params):
        """
        Constructor for the PivotFixtureSkill class.
        :param node: The node instance.
        :param id: The ID of the skill.
        :param params: A dictionary containing the parameters for the skill.

        params is a dictionary with the following keys:
        - "turning_dir_sliding" int: Direction of sliding (0 or 1).
        - "desired_pose" [x,y,z]: Desired pose for sliding.
        - "gripper_vel" int (0-100): Gripper velocity.
        - "gripper_force" int (0-100): Gripper force.
        - "gripper_direction" bool: Gripper direction (True or False).
        - "pos_next_fixture" [x,y,z]: Position of the next fixture.
        - "z_down" int: Z-axis down position.
        """
        super().__init__("Pivot_fixture_skill")
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
        self.add_state(eTaSL_StateMachine("pivotAligning","PivotAligning",
                                                        lambda bb: {
                                                            "pos_next_fixture": params["pos_next_fixture"],
                                                            "z_down": params["z_down"]
                                                        },
                                                        node=node))
        self.add_state(eTaSL_StateMachine("cablePivoting","CablePivoting",
                                                        lambda bb: {
                                                            "frame_next_fixture": params["frame_next_fixture"],
                                                            "turning_dir_pivoting": params["turning_dir_pivoting"],
                                                        },
                                                        node=node))
        self.add_state(MoveGripperToPosition(finger_position = params["cable_slide_pos"], gripping_velocity = params["gripper_vel"], node=node))


def test_gripper(node=None):
    """
    """
    return  Sequence("gripper_test", children=[
                                        GripPart(gripping_velocity=15.0, gripping_force=80.0, gripping_direction=True, node=node),
                                        # MoveGripperToPosition(finger_position=40.0, gripping_velocity=50.0, node=node),
                                        # MoveGripperToPosition(finger_position=0.0, gripping_velocity=50.0, node=node),
                                        # MoveGripperToPosition(finger_position=40.0, gripping_velocity=50.0, node=node)
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