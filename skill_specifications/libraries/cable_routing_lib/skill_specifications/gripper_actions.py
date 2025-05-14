from schunk_egu_egk_gripper_interfaces.action import MoveToAbsolutePosition, GripWithVelocity
from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *
from rclpy.duration import Duration

class MoveGripperToPosition(ActionClientBTFSM):
    def __init__(self, finger_position:float, gripping_velocity:float,
                     timeout:Duration = Duration(seconds=5.0), node=None):
        super().__init__(
            name="MoveGripperToPosition",
            action_name="/move_to_absolute_position",
            action_type=MoveToAbsolutePosition,
            outcomes=[],
            timeout=timeout,
            node=node
        )
        self.finger_position = finger_position
        self.gripping_velocity = gripping_velocity

    def fill_in_goal(self, blackboard, goal=None):
        goal = MoveToAbsolutePosition.Goal()
        goal.absolute_position = self.finger_position
        goal.velocity_of_movement = self.gripping_velocity
        return goal

    def process_result(self, blackboard, result) -> str:
        # blackboard.set("final_position", result.absolute_position)
        # if result.position_reached:
        #     return "POSITION_REACHED"
        # return "POSITION_FAILED"
        return SUCCEED

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        get_logger("action").info(f"Feedback: pos={fb.absolute_position:.2f}, vel={fb.velocity_of_movement:.2f}, current={fb.motor_current:.2f}")
        print(f"Feedback: pos={fb.absolute_position:.2f}, vel={fb.velocity_of_movement:.2f}, current={fb.motor_current:.2f}")

class GripPart(ActionClientBTFSM):
    def __init__(self, gripping_velocity:float, gripping_force:float, gripping_direction:bool,
                     timeout:Duration = Duration(seconds=5.0), node=None):
        super().__init__(
            name="GripPart",
            action_name="/grip",
            action_type=GripWithVelocity,
            outcomes=[],
            timeout=timeout,
            node=node
        )
        print("GripPart")
        self.gripping_velocity = gripping_velocity
        self.gripping_force = gripping_force
        self.gripping_direction = gripping_direction

    def fill_in_goal(self, blackboard, goal=None):
        goal = GripWithVelocity.Goal()
        goal.velocity_of_movement = self.gripping_velocity
        goal.gripping_force = self.gripping_force
        goal.grip_direction = self.gripping_direction
        return goal
    
    def process_result(self, blackboard, result) -> str:
        # blackboard.set("final_position", result.absolute_position)
        # if result.position_reached:
        #     return "POSITION_REACHED"
        # return "POSITION_FAILED"
        return SUCCEED