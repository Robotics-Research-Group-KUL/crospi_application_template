import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
# from betfsm_interfaces.action import Task
# from betfsm_interfaces.action import KulArmControl as Task
from keba_pilot_interfaces.action import KulArmControl as Task
from rclpy.executors import MultiThreadedExecutor

class ExecuteSkillActionServer(Node):
    def __init__(self):
        super().__init__('execute_skill_action_server')
        self.get_logger().info("Initializing ExecuteSkillActionServer...")

        # Create the action server
        self._action_server = ActionServer(
            self,
            Task,
            '/ai_prism/move_arm',
            self.execute_callback
        )

        self.get_logger().info("Action Server '/ai_prism/move_arm2' is now running!")

    def execute_callback(self, goal_handle):
        skill = goal_handle.request.skill
        parameters = goal_handle.request.parameters

        self.get_logger().info(f"Received goal - Executing skill: {skill} with parameters: {parameters}")

        # Simulate processing (optional)
        feedback_msg = Task.Feedback()
        feedback_msg.state = f"Processing skill: {skill}..."
        goal_handle.publish_feedback(feedback_msg)

        result = Task.Result()
        result.outcome = "success"
        
        self.get_logger().info(f"Successfully executed skill: {skill}")

        goal_handle.succeed()
        return result

def main(args=None):
    print("Starting ExecuteSkillActionServer...")
    rclpy.init(args=args)

    node = ExecuteSkillActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info("Spinning action server...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down action server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Action server shut down.")

if __name__ == '__main__':
    main()
