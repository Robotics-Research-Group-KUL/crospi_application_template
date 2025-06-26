# from skill_specifications.libraries.cable_routing_lib.skill_specifications.betfsm_channel_fixture_skill import ChannelFixtureSkill
# from skill_specifications.libraries.cable_routing_lib.skill_specifications.betfsm_pivot_fixture_skill import PivotFixtureSkill
# from betfsm.betfsm import *
# from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *


from skill_specifications.libraries.cable_routing_lib.skill_specifications.betfsm_initial_skills import GoToCableStart

import rclpy
import json
import route_utils as cu
import numpy as np

from scipy.spatial.transform import Rotation as R
import sys

import matplotlib.pyplot as plt

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

    def set_outcome(self, outcome):
        with self.outcome_lock:
            self.outcome = outcome

    def get_outcome(self):
        with self.outcome_lock:
            outcome = self.outcome
        return outcome


def generate_routing_sm(board_model, route_task_model, params):
    """
    Generate the routing state machine for the given board model and route task model.
    :param board_model: The board model.
    :param route_task_model: The route task model.
    :param params: The parameters for the routing state machine.
    :return: The routing state machine.
    """
    routing_sm = []

    tangent_lines = []
    waypoints = []
    # TODO: Find a good way to initialze the prev_segment_dir
    prev_segment_dir = 1
    route_task_model["Directions"] = []

    channel_dist_min = params["channel_dist_min"]
    channel_dist_delta = params["channel_dist_delta"]
    slack = params["slack"]

    for i in range(len(route_task_model["Route"])-2):
        skill_params = {}
        centered_i = i + 1
        prev_fixture_id = route_task_model["Route"][centered_i-1]
        current_fixture_id = route_task_model["Route"][centered_i]
        next_fixture_id = route_task_model["Route"][centered_i+1]

        prev_fixture = board_model["Fixtures"][prev_fixture_id]
        current_fixture = board_model["Fixtures"][current_fixture_id]
        next_fixture = board_model["Fixtures"][next_fixture_id]

        direction_segment = cu.compute_direction(prev_fixture, current_fixture, next_fixture, prev_segment_dir)
        route_task_model["Directions"].append(direction_segment)

        tangent = cu.compute_tangent(prev_fixture, current_fixture, d1 = prev_segment_dir, d2 = direction_segment, dilate_2 = True)
        
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
        
        # if current_fixture["type"] == "PIVOT":
        #     skill_params["turning_dir_sliding"] = direction_segment
        #     skill_params["desired_pos"] = waypoint
        #     skill_params["gripper_vel"] = params["gripper_vel"]
        #     skill_params["gripper_force"] = params["gripper_force"]
        #     skill_params["gripper_direction"] = params["gripper_direction"]
        #     # skill_params["z_down"]
        #     # skill_params["frame_next_fixture"]
        #     # skill_params["turning_dir_pivoting"]
        #     routing_sm.append(PivotFixtureSkill(node=None, id=current_fixture_id, skill_params=skill_params))

        # elif current_fixture["type"] == "CHANNEL":
        #     skill_params["turning_dir_sliding"] = direction_segment
        #     skill_params["desired_pos"] = waypoint
        #     skill_params["gripper_vel"] = params["gripper_vel"]
        #     skill_params["gripper_force"] = params["gripper_force"]
        #     skill_params["gripper_direction"] = params["gripper_direction"]
        #     # skill_params["channel_aligning_pose"]
        #     # skill_params["channel_inserting_pose"]
        #     # skill_params["cable_slide_pos"]
        #     routing_sm.append(ChannelFixtureSkill(node=None, id=current_fixture_id, skill_params=skill_params))

    fig, ax = plt.subplots()
    cu.plot_board(ax, board_model,tangent_lines,[],waypoints)
    # save the figure
    plt.savefig("waypoints.pdf")

    return routing_sm

# TODO: Change this to allow to add multiple tasks. This is defined in etasl_params.py in etasl_ros2_py
# def load_task_list( json_file_name: str, blackboard: dict) -> None:
#     """
#     Loads a task list from a file. References to packages and environment variables in the name
#     are expanded (using the expand_... functions)

#     Parameters:
#         json_file_name: 
#             json file containing the task list.
#         blackboard:
#             blackboard into which to load the task list.
    
#     Returns:
#         None
#     """
#     with open(expand_ref(json_file_name), 'r') as json_file:
#         parameters = json.load(json_file)
#         blackboard["tasks"] = parameters["tasks"]

# TODO: Generate the application fsm
def generate_application_fsm(route_task_model, board_model, params):
    """
    Generate the application fsm for the given route task model and board model.
    :param route_task_model: The route task model.
    :param board_model: The board model.
    :param params: The parameters for the application fsm.
    :return: The application fsm.
    """
    list_of_skills = []
    for i in range(len(route_task_model["Route"])-2):
        print(i)
    return None


def main(args=None):

    # rclpy.init(args=args)
    # my_node = BeTFSMNode.get_instance("pcb_insertion_skill")

    # set_logger("default",my_node.get_logger())
    # #set_logger("service",my_node.get_logger())
    # #set_logger("state",my_node.get_logger())

    route_name = "test_route"
    route_path = "/home/robpc/robelix_ws/src/neura-gui/saved_routes/{}.json".format(route_name)

    params_path = "parameters/application_params.json"

    # load the route
    with open(route_path, 'r') as f:
        route = json.load(f)

    # load the parameters
    with open(params_path, 'r') as f:
        params = json.load(f)

    # blackboard = {}
    # # load the tasks to the blackboard
    # load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/cable_routing_lib/tasks/channel_fixture_skill.json",blackboard)
    # load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/cable_routing_lib/tasks/pivot_fixture_skill.json",  blackboard)
    # load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/cable_routing_lib/tasks/initial_skills.json", blackboard)

    # # load the application parameters to the blackboard
    # blackboard["application_params"] = params

    # Split the route and the board model
    board_model = route
    route_task_model = {}
    route_task_model["Route"] = list(route["Fixtures"].keys())

    # add additional parametes for the fixtures in the board model, if their type is "CHANNEL" add width and height, if their type is "PIVOT" add the radius and dilated_radius
    for fixture_id in board_model["Fixtures"]:
        fixture = board_model["Fixtures"][fixture_id]
        if fixture["type"] == "CHANNEL":
            fixture["width"] = params["channel_width"]
            fixture["height"] = params["channel_height"]
        elif fixture["type"] == "PIVOT":
            fixture["radius"] = params["pivot_radius"]
            fixture["dilated_radius"] = params["pivot_dilated_radius"]


    # Board transformation matrix this should come from the vision system
    T_base_board = np.eye(4)
    T_base_board[:3, :3] = R.from_euler('z', -np.pi/2).as_matrix()  # Rotation around z-axis
    T_base_board[0:3, 3] = [0.0, -0.13, -0.045]  # Translation vector
    
    _ = generate_routing_sm(board_model, route_task_model, params)

    # # prints a graphviz representation of sm:
    # vis = GraphViz_Visitor()
    # sm.accept(vis)
    # vis.print()
    
    # sm = GoToCableStart(node=my_node, id="go_to_cable_start")

    # runner = BeTFSMRunner(my_node,sm,blackboard,0.01)

    # rclpy.spin(my_node)
        
    # print("shutdown")


if __name__ == "__main__":
    sys.exit(main(sys.argv))