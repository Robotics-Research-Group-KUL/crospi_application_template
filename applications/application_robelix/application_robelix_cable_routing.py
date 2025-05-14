# from skill_specifications.libraries.cable_routing_lib.skill_specifications.betfsm_channel_fixture_skill import ChannelFixtureSkill
# from skill_specifications.libraries.cable_routing_lib.skill_specifications.betfsm_pivot_fixture_skill import PivotFixtureSkill
# from betfsm.betfsm import *
# from betfsm.betfsm_ros import *
# from betfsm.betfsm_etasl import *
# from betfsm.betfsm_action_server import *

import json
import route_utils as cu
import numpy as np

route_name = "test_route"
route_path = "/home/robpc/robelix_ws/src/neura-gui/saved_routes/{}.json".format(route_name)

params_pad = "parameters/application_params.json"

# load the route
with open(route_path, 'r') as f:
    route = json.load(f)

# load the parameters
with open(params_pad, 'r') as f:
    params = json.load(f)

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

channel_dist_min = params["channel_dist_min"]
channel_dist_delta = params["channel_dist_delta"]
slack = params["slack"]

tangent_lines = []
waypoints = []
prev_segment_dir = 1
route_task_model["Directions"] = []
for i in range(len(route_task_model["Route"])-2):
    centered_i = i + 1
    prev_fixture_id = route_task_model["Route"][centered_i-1]
    current_fixture_id = route_task_model["Route"][centered_i]
    next_fixture_id = route_task_model["Route"][centered_i+1]

    prev_fixture = board_model["Fixtures"][prev_fixture_id]
    current_fixture = board_model["Fixtures"][current_fixture_id]
    next_fixture = board_model["Fixtures"][next_fixture_id]

    direction_segment = cu.compute_direction(prev_fixture, current_fixture, next_fixture)

    tangent = cu.compute_tangent(prev_fixture, current_fixture, d1 = prev_segment_dir, d2 = direction_segment, dilate_2 = True)
    
    tangent_lines.append(tangent)
    prev_segment_dir = direction_segment
    # pdb.set_trace()
    unit_tangent_vector = np.array(np.array(tangent[1]) - np.array(tangent[0]))/np.linalg.norm(np.array(tangent[1]) - np.array(tangent[0]))
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

        # segment["rz"] = orientation
        slack = channel_dist_min + channel_dist_delta*(1-cos_or)

    waypoint = tangent[1] + slack*unit_tangent_vector
    waypoints.append(waypoint)

    # print("i: ", direction_segment)
    route_task_model["Directions"].append(direction_segment)

print("Route task model: ", route_task_model)


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