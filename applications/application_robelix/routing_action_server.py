import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from betfsm_interfaces.action import Task
from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *
from rclpy.executors import MultiThreadedExecutor

from etasl_ros2_py import etasl_params
from matplotlib import pyplot as plt

import route_utils as ru
import json
import numpy as np

from scipy.spatial.transform import Rotation as R

from skill_specifications.libraries.cable_routing_lib.skill_specifications.betfsm_channel_fixture_skill import ChannelFixtureSkill
from skill_specifications.libraries.cable_routing_lib.skill_specifications.betfsm_pivot_fixture_skill import PivotFixtureSkill, BeTFSMRunner


# TODO: Implement goal callback to reject if there is already a routing task running
# TODO: Implement feedback, it could be easily handle by adding a feedbackState from BetFSM at the end of each skill
class RoutingActionServer(Node):

    def __init__(self):
        super().__init__('routing_action_server')
        self._action_server = ActionServer(
            self,
            Task,
            'RoutingAction',
            self.execute_callback)
        
        self.get_logger().info('Routing Action Server has been started.')

        self.declare_parameter('parameter_path')
        # self.my_node = BeTFSMNode.get_instance("routing_action_server")

        self.state_machine = None

        # Load parameters from the JSON file
        parameter_path = self.get_parameter('parameter_path').get_parameter_value().string_value
        if not parameter_path:
            parameter_path = '/home/robpc/robelix_ws/src/etasl_ros2_application_template/applications/application_robelix/parameters/application_params.json'
        try:
            with open(parameter_path, 'r') as f:
                self.params = json.load(f)
                self.get_logger().info(f"Loaded parameters: {self.params}")
        except FileNotFoundError:
            self.get_logger().error(f"Parameter file not found: {parameter_path}")
        
        self.blackboard = {"tasks": []}
        self.load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/cable_routing_lib/tasks/channel_fixture_skill.json")
        self.load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/cable_routing_lib/tasks/pivot_fixture_skill.json")
        self.load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/cable_routing_lib/tasks/initial_skills.json")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing cable routing task ...')
        self.get_logger().info(f"Received task: {goal_handle.request.task}")
        self.get_logger().info(f"Received route: {goal_handle.request.parameters}")

        result = Task.Result()

        parameters_str = goal_handle.request.parameters
        board_model, route_task_model = self.get_board_and_route(parameters_str)

        intial_sm = eTaSL_StateMachine("MovingHome", "MovingHome", node=self)
        routing_sm = self.generate_routing_sm(board_model, route_task_model)

        rate = self.create_rate(100)
        rate.sleep()

        intial_sm.reset()
        outcome=intial_sm(self.blackboard)
        while outcome==TICKING:
            rate.sleep()
            outcome=intial_sm(self.blackboard)
        self.get_logger().info(f'Finished state machine {goal_handle.request.task} with outcome {outcome}')

        goal_handle.succeed()  # Mark the goal as succeeded
        return result
    
    def load_task_list(self, json_file_name: str) -> None:
        """
        Loads a task list from a file. References to packages and environment variables in the name
        are expanded (using the expand_... functions)

        Parameters:
            json_file_name: 
                json file containing the task list.
            blackboard:
                blackboard into which to load the task list.
        
        Returns:
            None
        """
        with open(etasl_params.expand_ref(json_file_name), 'r') as json_file:
            parameters = json.load(json_file)
            # print(parameters["tasks"])
            self.blackboard["tasks"] += parameters["tasks"]
            # import pdb; pdb.set_trace()
    
    def get_board_and_route(self, parameters_str):
        """
        Extract the board model and route task model from the parameters string.
        :param parameters_str: The parameters string containing the board and route information.
        :return: A tuple containing the board model and route task model.
        """
        parameters_dict = json.loads(parameters_str)
        board_model = {"Fixtures": parameters_dict["route"]}
        route_task_model = {}
        route_task_model["Route"] = list(parameters_dict["route"].keys())

        for fixture_id in board_model["Fixtures"]:
            fixture = board_model["Fixtures"][fixture_id]
            if fixture["type"] == "CHANNEL":
                fixture["width"] = self.params["channel_width"]
                fixture["height"] = self.params["channel_height"]
            elif fixture["type"] == "PIVOT":
                fixture["radius"] = self.params["pivot_radius"]
                fixture["dilated_radius"] = self.params["pivot_dilated_radius"]

        return board_model, route_task_model
    

    def generate_routing_sm(self, board_model, route_task_model):
        """
        Generate the routing state machine for the given board model and route task model.
        :param board_model: The board model.
        :param route_task_model: The route task model.
        :return: The routing state machine.
        """
        routing_sm = []

        tangent_lines = []
        waypoints = []
        # TODO: Find a good way to initialze the prev_segment_dir
        prev_segment_dir = 1
        route_task_model["Directions"] = []

        channel_dist_min = self.params["channel_dist_min"]
        channel_dist_delta = self.params["channel_dist_delta"]
        slack = self.params["slack"]

        for i in range(len(route_task_model["Route"])-2):
            skill_params = {}
            centered_i = i + 1
            prev_fixture_id = route_task_model["Route"][centered_i-1]
            current_fixture_id = route_task_model["Route"][centered_i]
            next_fixture_id = route_task_model["Route"][centered_i+1]

            prev_fixture = board_model["Fixtures"][prev_fixture_id]
            current_fixture = board_model["Fixtures"][current_fixture_id]
            next_fixture = board_model["Fixtures"][next_fixture_id]

            direction_segment = ru.compute_direction(prev_fixture, current_fixture, next_fixture, prev_segment_dir)
            route_task_model["Directions"].append(direction_segment)

            tangent = ru.compute_tangent(prev_fixture, current_fixture, d1 = prev_segment_dir, d2 = direction_segment, dilate_2 = True)
            
            tangent_lines.append(tangent)
            prev_segment_dir = direction_segment
            # pdb.set_trace()
            unit_tangent_vector = np.array(np.array(tangent[1]) - np.array(tangent[0]))/np.linalg.norm(np.array(tangent[1]) - np.array(tangent[0]))

            # Modify the slack for channels based on the orientation difference between the tangent vector and the channel fixture orientation
            if current_fixture["type"] == "CHANNEL":
                orientation_1 = current_fixture["rz"]
                orientation_2 = current_fixture["rz"] + np.pi

                dot_product_orientation_1 = np.dot(unit_tangent_vector,np.array([np.cos(orientation_1), np.sin(orientation_1)]))
                dot_product_orientation_2 = np.dot(unit_tangent_vector,np.array([np.cos(orientation_2), np.sin(orientation_2)]))

                if dot_product_orientation_1 >= dot_product_orientation_2:
                    orientation = orientation_1
                    cos_or = dot_product_orientation_1
                else:
                    orientation = orientation_2
                    cos_or = dot_product_orientation_2
                
                channel_rz = orientation
                slack = channel_dist_min + channel_dist_delta*(1-cos_or)

            waypoint = tangent[1] + slack*unit_tangent_vector
            waypoints.append(waypoint)

            waypoint_x, waypoint_y = waypoint
            waypoint_z = self.params["slide_height"]

            start_fixture_frame = np.eye(4)
            start_fixture_frame[:3, 3] = np.array([prev_fixture["x"], prev_fixture["y"], self.params["slide_height"]])

            current_fixture_frame = np.eye(4)
            current_fixture_frame[:3, 3] = np.array([current_fixture["x"], current_fixture["y"], self.params["slide_height"]])

            start_fixture_frame_wrt_current_fixture_frame = start_fixture_frame @ np.linalg.inv(current_fixture_frame)

            skill_params["turning_dir_sliding"] = direction_segment
            skill_params["desired_pos"] = [waypoint_x, waypoint_y, waypoint_z]
            skill_params["gripper_vel"] = self.params["gripper_vel"]
            skill_params["gripper_force"] = self.params["gripper_force"]
            skill_params["gripper_direction"] = self.params["gripper_direction"]
            skill_params["cable_slide_pos"] = self.params["cable_slide_pos"]
            
            if current_fixture["type"] == "PIVOT":
                skill_params["pos_previous_fixture"] = [start_fixture_frame_wrt_current_fixture_frame[0,3], start_fixture_frame_wrt_current_fixture_frame[1,3], 
                                                                start_fixture_frame_wrt_current_fixture_frame[2,3]]
                
                skill_params["z_down"] = self.params["pivot_z_down"]

                skill_params["frame_next_fixture"] = [next_fixture["x"], next_fixture["y"], self.params["slide_height"],
                                                   0, 0, 0, 1]

                # print(f"Fixture {current_fixture_id} parameters: {skill_params}")
                
                routing_sm.append(PivotFixtureSkill(node=self, id=current_fixture_id, skill_params=skill_params))

            if current_fixture["type"] == "CHANNEL":
                quaternion = R.from_euler('z', channel_rz).as_quat()
                skill_params["channel_aligning_pose"] = [current_fixture["x"], current_fixture["y"], self.params["slide_height"],
                                                            quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
                skill_params["channel_inserting_pose"] = [current_fixture["x"], current_fixture["y"], self.params["channel_height"] - self.params["channel_insertion_diff"],
                                                            quaternion[0], quaternion[1], quaternion[2], quaternion[3]]

                # print(f"Fixture {current_fixture_id} parameters: {skill_params}")
                
                routing_sm.append(ChannelFixtureSkill(node=self, id=current_fixture_id, skill_params=skill_params))

        fig, ax = plt.subplots()
        ru.plot_board(ax, board_model,tangent_lines,[],waypoints)
        # save the figure
        plt.savefig("waypoints.pdf")

        return Sequence("CableRouting", children = routing_sm)


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()  
    

    routing_action_server = RoutingActionServer()

    executor.add_node(routing_action_server)    
    executor.spin()
    
    # action_server.destroy()
    rclpy.shutdown()

    # rclpy.spin(routing_action_server)


if __name__ == '__main__':
    main()