import numpy as np
import matplotlib.pyplot as plt
import json
import pdb
from scipy.interpolate import CubicSpline
# from shapely.geometry import Polygon, Point
from scipy.integrate import quad

def compute_direction(f1, f2, f3):
    angle_1 = np.arctan2(f2["y"] - f1["y"], f2["x"] - f1["x"])
    angle_2 = np.arctan2(f3["y"] - f2["y"], f3["x"] - f2["x"])
    # print("angle_1: ", angle_1)
    # print("angle_2: ", angle_2)
    angle_diff = angle_2 - angle_1
    # print("angle_diff: ", angle_diff)
    if np.abs(angle_diff) <= np.pi:
        dir = np.sign(angle_diff)
    else:
        dir = -np.sign(angle_diff)
    return dir

def compute_tangent(fixture_1, fixture_2, d1, d2,
                    dilate_1: bool = False, dilate_2: bool = False):
    if fixture_1["type"] != "CIRCLE":
        r1 = 0.0
    else:
        if dilate_1:
            r1 = fixture_1["dilated_radius"]
        else:
            r1 = fixture_1["radius"]
    
    if fixture_2["type"] != "CIRCLE":
        r2 = 0.0
    else:
        if dilate_2:
            r2 = fixture_2["dilated_radius"]
        else:
            r2 = fixture_2["radius"]
    
    gamma = np.arctan2(fixture_2["y"] - fixture_1["y"], fixture_2["x"] - fixture_1["x"])
    beta = np.arcsin((d1*r1 - d2*r2) / np.linalg.norm([fixture_2["x"] - fixture_1["x"], fixture_2["y"] - fixture_1["y"]]))
    alpha =(gamma + beta)

    o_x = fixture_1["x"] + d1*r1*np.sin(alpha)
    o_y = fixture_1["y"] - d1*r1*np.cos(alpha)

    p_x = fixture_2["x"] + d2*r2*np.sin(alpha)
    p_y = fixture_2["y"] - d2*r2*np.cos(alpha)

    return (o_x, o_y), (p_x, p_y)

def create_circular_fixture(x, y, r):
    return {"type": "CIRCLE", "x": x, "y": y, "radius": r}

def cubic_bezier_curve(t, P0, P1, P2, P3):
    return (1 - t)**3 * P0 + 3 * (1 - t)**2 * t * P1 + 3 * (1 - t) * t**2 * P2 + t**3 * P3

# Define the derivative of the cubic Bézier curve
def cubic_bezier_derivative(t, P0, P1, P2, P3):
    return (3 * (1 - t)**2 * (P1 - P0) +
            6 * (1 - t) * t * (P2 - P1) +
            3 * t**2 * (P3 - P2))

# Define the quadratic Bézier curve function
def quadratic_bezier(t, P0, P1, P2):
    return (1 - t)**2 * P0 + 2 * (1 - t) * t * P1 + t**2 * P2

# Define the function to integrate (the magnitude of the derivative)
def curve_speed(t, argdict):
    P0 = argdict["P0"]
    P1 = argdict["P1"]
    P2 = argdict["P2"]
    P3 = argdict["P3"]
    dB_dt = cubic_bezier_derivative(t, P0, P1, P2, P3)
    return np.linalg.norm(dB_dt)

def compute_approaching_zone_points(n_points, fixture_1, fixture_2, d1, d2, tool_radius, slack, dilation):
    if fixture_1["type"] != "CIRCLE":
        r1 = 0.0
    else:
        r1 = fixture_1["radius"]

    if fixture_2["type"] != "CIRCLE":
        r2 = 0.0
    else:
        r2 = fixture_2["radius"]

    p1 = (fixture_1["x"], fixture_1["y"])
    p2 = (fixture_2["x"], fixture_2["y"])
    
    # Tangents
    t1, t2 = compute_tangent(fixture_1, fixture_2, d1, d2)
    undilated_tangent_vector = np.array([t2[0] - t1[0], t2[1] - t1[1]])
    unit_undilated_tangent_vector = undilated_tangent_vector/np.linalg.norm(undilated_tangent_vector)
    # Compute the end point of the cable
    undilated_end_point = unit_undilated_tangent_vector*(slack+tool_radius) + t2
    # Compute distance from fixture to end point
    length = np.linalg.norm(undilated_end_point - t1)
    points = []
    for dila_i in np.linspace(dilation, 0, n_points):
        fixture_2_dilated = create_circular_fixture(p2[0], p2[1], r2+dila_i)
        t1_dilated, t2_dilated = compute_tangent(fixture_1, fixture_2_dilated, d1, d2)
        dilated_tangent_vector = np.array([t2_dilated[0] - t1_dilated[0], t2_dilated[1] - t1_dilated[1]])
        unit_dilated_tangent_vector = dilated_tangent_vector/np.linalg.norm(dilated_tangent_vector)

        angle = np.arccos(min(np.dot(unit_undilated_tangent_vector, unit_dilated_tangent_vector),1.0))
        cable_arc = r1*angle

        zone_1_point = unit_dilated_tangent_vector*(length + tool_radius + (d1*d2)*cable_arc) + t1_dilated
        points.append(zone_1_point)
    points = np.array(points)
    return points

def compute_turning_zone_points(n_points, fixture_2, fixture_3, d2, tool_radius, 
                                cable_length, theta_i, slack, align_tolerance):
    if fixture_2["type"] != "CIRCLE":
        r2 = 0.0
    else:
        r2 = fixture_2["radius"]

    if fixture_3["type"] != "CIRCLE":
        r3 = 0.0
    else:
        r3 = fixture_3["radius"]

    p2 = (fixture_2["x"], fixture_2["y"])
    p3 = (fixture_3["x"], fixture_3["y"])

    theta_f = np.arctan2(p3[1]-p2[1], p3[0]-p2[0])
    theta_f = theta_f + -1*(np.sign(theta_f)-1)*np.pi
    # Ensure the arc goes in d2 direction by wrapping the angle
    if theta_f < theta_i:
        theta_f += (np.sign(d2)+1)*np.pi
    else:
        theta_i += (np.sign(-d2)+1)*np.pi

    zone_2_points = []

    # pdb.set_trace()
    
    for theta in np.linspace(theta_i, theta_f, num=n_points):
        beta_tangent = theta + (np.sign(d2)+1)*np.pi/2 + np.pi/2 #pi/2 if d2 is positive, 3*pi/2 if d2 is negative
        tangent_next_fixture_point = (p2[0] + r2*np.cos(beta_tangent), p2[1] + r2*np.sin(beta_tangent))
        next_fixture_tanget_vector = d2*np.array([-np.sin(beta_tangent),np.cos(beta_tangent)])

        unit_next_fixture_tangent_vector = next_fixture_tanget_vector/np.linalg.norm(next_fixture_tanget_vector)
        cable_arc = r2*np.abs(theta-theta_i)
        zone_2_point = unit_next_fixture_tangent_vector*(cable_length + tool_radius - cable_arc) + tangent_next_fixture_point
        zone_2_points.append(zone_2_point)

    last_tool_cp = unit_next_fixture_tangent_vector*(cable_length - cable_arc) + tangent_next_fixture_point

    zone_2_points = np.array(zone_2_points)

    return zone_2_points, last_tool_cp

def generate_arc_points(center, radius, point1, point2, d2, num_points=100):
    # Extract the center and the points
    cx, cy = center
    x1, y1 = point1
    x2, y2 = point2
    
    # Calculate the angles from the center to the points
    angle1 = np.arctan2(y1 - cy, x1 - cx)
    angle2 = np.arctan2(y2 - cy, x2 - cx)

    angle1 = angle1 + -1*(np.sign(angle1)-1)*np.pi
    angle2 = angle2 + -1*(np.sign(angle2)-1)*np.pi

    # Ensure the arc goes in d2 direction by wrapping the angle
    if angle2 < angle1:
        angle2 += (np.sign(d2)+1)*np.pi
    else:
        angle1 += (np.sign(-d2)+1)*np.pi
    
    # pdb.set_trace()
    # Create an array of angles from angle1 to angle2
    angles = np.linspace(angle1, angle2, num_points)
    
    # Generate the points along the arc
    arc_points = [(cx + radius * np.cos(angle), cy + radius * np.sin(angle)) for angle in angles]
    arc_points = np.array(arc_points)
    
    return arc_points

# def get_freespace_polygon(fixture_1: dict, fixture_2: dict, fixture_3: dict, d1: int, d2: int, 
#                 tool_radius: float, slack: float, dilation: float, align_tolerance: float, n_points_zone=100) -> Polygon:

#     if fixture_1["type"] != "CIRCLE":
#         r1 = 0.0
#     else:
#         r1 = fixture_1["radius"]
#     if fixture_2["type"] != "CIRCLE":
#         r2 = 0.0
#     else:
#         r2 = fixture_2["radius"]

#     p2 = (fixture_2["x"], fixture_2["y"])

#     t1, t2 = compute_tangent(fixture_1, fixture_2, d1, d2)
#     undilated_tangent_vector = np.array([t2[0] - t1[0], t2[1] - t1[1]])
#     unit_undilated_tangent_vector = undilated_tangent_vector/np.linalg.norm(undilated_tangent_vector)
#     undilated_end_point = unit_undilated_tangent_vector*(slack+tool_radius) + t2
#     length = np.linalg.norm(undilated_end_point - t1)

#     t1_dilated, t2_dilated = compute_tangent(fixture_1, fixture_2, d1, d2, dilate_2=True)
#     dilated_tangent_vector = np.array([t2_dilated[0] - t1_dilated[0], t2_dilated[1] - t1_dilated[1]])
#     unit_dilated_tangent_vector = dilated_tangent_vector/np.linalg.norm(dilated_tangent_vector)

#     # Where is the sign of turning comming from to know if you need to substract or to add the arc?
#     cable_arc = r1*np.arccos(np.dot(unit_undilated_tangent_vector, unit_dilated_tangent_vector))
#     dilated_end_point = unit_dilated_tangent_vector*(length + (d1*d2)*cable_arc) + t1_dilated

#     points = compute_approaching_zone_points(n_points_zone, fixture_1, fixture_2, d1, d2, tool_radius, slack, dilation)

#     # Compute atan2 between p2 and p3
#     theta_i = np.arctan2(unit_undilated_tangent_vector[1],unit_undilated_tangent_vector[0])
#     cable_length = np.linalg.norm(t2 - undilated_end_point)
#     zone_2_points, last_tool_cp = compute_turning_zone_points(n_points_zone, fixture_2, fixture_3, d2, tool_radius, 
#                                                                 cable_length, theta_i, slack, align_tolerance)
#     p2_tool_ = last_tool_cp - p2
#     p2_tool = p2_tool_/np.linalg.norm(p2_tool_)

#     last_tool_cp_fixture = create_circular_fixture(last_tool_cp[0], last_tool_cp[1], 0)
#     _t_fixture, _t_tool  = compute_tangent(fixture_2, last_tool_cp_fixture, d2, d2)
#     t_vector = np.array([_t_tool[0] - _t_fixture[0], _t_tool[1] - _t_fixture[1]])
#     t_unit_vector = t_vector/np.linalg.norm(t_vector)
#     cable_end_conf_1 = _t_fixture
#     cable_end_conf_2 = -t_unit_vector*tool_radius + last_tool_cp

#     dummy_fixture_tool = create_circular_fixture(dilated_end_point[0], dilated_end_point[1], tool_radius)
#     tangent_dilated_point = create_circular_fixture(t2_dilated[0], t2_dilated[1], 0)
#     t3, t4 = compute_tangent(tangent_dilated_point, dummy_fixture_tool, d2, d2)
#     first_tool_arc = generate_arc_points(dilated_end_point, tool_radius, t4, points[0], d2, num_points=n_points_zone)
#     second_tool_arc = generate_arc_points(last_tool_cp, tool_radius, zone_2_points[-1], cable_end_conf_2, d2, num_points=n_points_zone)
#     fixture_arc = generate_arc_points(p2, r2, t2, cable_end_conf_1, d2, num_points=n_points_zone)

#     polygon_points = np.concatenate(([t1_dilated], [t2_dilated], [t3], [t4], first_tool_arc, points, zone_2_points, second_tool_arc, [cable_end_conf_2], [cable_end_conf_1], np.flip(fixture_arc,axis=0), [t2], [t1]))

#     polygon_area = Polygon(polygon_points)

#     return polygon_area

# def get_freespace_polygon_channel(fixture_1: dict, fixture_2: dict, fixture_3: dict, d1: int, d2: int, 
#                 tool_radius: float, slack: float, dilation: float, n_points_zone=100, ax=None) -> Polygon:

#     tangent_weight = 0.07
#     rz = fixture_2["rz"]
#     channel_p = (fixture_2["x"], fixture_2["y"])
    
#     if fixture_1["type"] != "CIRCLE":
#         r1 = 0.0
#     else:
#         r1 = fixture_1["radius"]
    
#     t1, t2 = compute_tangent(fixture_1, fixture_2, d1, d2)
    
#     argdict = {"P0": np.array([t1[0], t1[1]]), 
#                "P3": np.array([t2[0], t2[1]])}

#     # Distance beween P0 and P3
#     dist = np.linalg.norm(argdict["P3"] - argdict["P0"])

#     # Middle point between P0 and P3
#     Pmid = (argdict["P0"] + argdict["P3"]) / 2
#     unit_v = (argdict["P3"] - argdict["P0"]) / np.linalg.norm(argdict["P3"] - argdict["P0"])
#     # Rotate unit vector 90 degrees
#     unit_v = np.array([unit_v[1], -unit_v[0]])
#     argdict["P2"] = argdict["P3"] - np.array([np.cos(rz), np.sin(rz)])*tangent_weight

#     # Iterate over the middle point to find the P1
#     i = 0
#     while True:
#         # Calculate the approximate length of the curve
#         argdict["P1"] = Pmid + i*unit_v
#         length, _ = quad(curve_speed, 0, 1, argdict)
#         if length >= dist + slack:
#             P1_low = argdict["P1"]
#             break
#         i+=0.001

#     i=0
#     while True:
#         # Calculate the approximate length of the curve
#         argdict["P1"] = Pmid - i*unit_v
#         length, _ = quad(curve_speed, 0, 1, argdict)
#         if length >= dist + slack:
#             P1_high = argdict["P1"]
#             break
#         i+=0.001

#     # Generate points on the curve
#     t_values = np.linspace(0, 1, n_points_zone)  # Parameter t ranges from 0 to 1
#     low_curve_points = np.array([cubic_bezier_curve(t, argdict["P0"], P1_low, argdict["P2"], argdict["P3"]) for t in t_values])
#     high_curve_points = np.array([cubic_bezier_curve(t, argdict["P0"], P1_high, argdict["P2"], argdict["P3"]) for t in t_values])

#     polygon_area_bending = Polygon(np.concatenate((low_curve_points, np.flip(high_curve_points,axis=0))))


#     unit_strech_vector = np.array([np.cos(rz), np.sin(rz)])

#     # tangent_vector = np.array([t2[0] - t1[0], t2[1] - t1[1]])
#     # unit_tangent_vector = tangent_vector/np.linalg.norm(tangent_vector)
#     # pdb.set_trace()
#     tool_intial_cp = unit_strech_vector*(tool_radius + fixture_2["width"]/2) + channel_p

#     tool_final_cp = unit_strech_vector*(tool_radius + fixture_2["width"]/2 + slack) + channel_p

#     dummy_fixture_tool_initial = create_circular_fixture(tool_intial_cp[0], tool_intial_cp[1], tool_radius)
#     dummy_fixture_tool_final = create_circular_fixture(tool_final_cp[0], tool_final_cp[1], tool_radius)

#     t_o_1, t_o_2 = compute_tangent(dummy_fixture_tool_initial, dummy_fixture_tool_final, 1, 1)
#     t_i_1, t_i_2 = compute_tangent(dummy_fixture_tool_initial, dummy_fixture_tool_final, -1, -1)

#     intial_points = generate_arc_points(tool_intial_cp, tool_radius, t_o_1, t_i_1, -1, num_points=n_points_zone)
#     final_points = generate_arc_points(tool_final_cp, tool_radius, t_o_2, t_i_2, 1, num_points=n_points_zone)

#     polygon_points = np.concatenate(([t_o_1], [t_o_2], final_points, [t_i_2], [t_i_1], np.flip(intial_points,axis=0)))

#     polygon_area = Polygon(polygon_points)

#     if ax:
#         circle1 = plt.Circle(tool_intial_cp, tool_radius, color='lime', fill=False)
#         circle2 = plt.Circle(tool_final_cp, tool_radius, color='lime', fill=False)

#         ax.add_patch(circle1)
#         ax.add_patch(circle2)

#         ax.plot([t_o_1[0], t_o_2[0]], [t_o_1[1], t_o_2[1]], color='k')
#         ax.plot([t_i_1[0], t_i_2[0]], [t_i_1[1], t_i_2[1]], color='k')

#         ax.plot(intial_points[:,0],intial_points[:,1], color='k')
#         ax.plot(final_points[:,0],final_points[:,1], color='k')

#         medium_curve = np.array([quadratic_bezier(t, argdict["P0"], argdict["P2"], argdict["P3"]) for t in t_values])
#         ax.plot(low_curve_points[:, 0], low_curve_points[:, 1], label='Bézier Curve', color='blue')
#         ax.plot(high_curve_points[:, 0], high_curve_points[:, 1], label='Bézier Curve', color='red')
#         ax.plot(medium_curve[:, 0], medium_curve[:, 1], label='Bézier Curve', color='green')

#         # Plot control points P0, P1, P2, and P3
#         ax.plot([argdict["P0"][0], P1_low[0]], [argdict["P0"][1], P1_low[1]], 'k--')
#         ax.plot([P1_low[0], argdict["P2"][0]], [P1_low[1], argdict["P2"][1]], 'k--')
#         ax.plot([argdict["P2"][0], argdict["P3"][0]], [argdict["P2"][1], argdict["P3"][1]], 'k--')

#         # Plot control points P0, P1, P2, and P3
#         ax.plot([argdict["P0"][0], P1_high[0]], [argdict["P0"][1], P1_high[1]], 'k--')
#         ax.plot([P1_high[0], argdict["P2"][0]], [P1_high[1], argdict["P2"][1]], 'k--')
#         ax.plot([argdict["P2"][0], argdict["P3"][0]], [argdict["P2"][1], argdict["P3"][1]], 'k--')

#         # pdb.set_trace()

#     return polygon_area_bending, polygon_area

def check_collisions(polygon, board_model):
    for fixture_id, fixture in board_model["Fixtures"].items():
        if fixture["type"] == "CIRCLE" or fixture["type"] == "CHANNEL":
            if polygon.intersects(fixture["polygon"]):
                print("Fixture {} intersects with polygon".format(fixture_id))
            else:
                print("Fixture {} does not intersect with polygon".format(fixture_id))

def create_circle(x,y,r):
    theta = np.linspace(0,2*np.pi,100)
    x_list = r*np.cos(theta)+x
    y_list = r*np.sin(theta)+y
    return (x_list, y_list)

def generate_tangents_point(point, fixture):
    tangents = []
    x0 = point['x']
    y0 = point['y']
    
    x1 = fixture['x']
    y1 = fixture['y']
    r1 = fixture['dilated_radius"]']

    delta_x = x1-x0
    delta_y = y1-y0
    distance = np.sqrt(delta_x**2 + delta_y**2)
    # k=1
    for k in [-1,1]:
        c1_x = -(r1**2)/(distance**2)*delta_x - (k*r1/distance**2)*np.sqrt(distance**2-r1**2)*delta_y
        c1_y = -(r1**2)/(distance**2)*delta_y + (k*r1/distance**2)*np.sqrt(distance**2-r1**2)*delta_x
        
        tangents.append(((x0, c1_x+x1), (y0, c1_y+y1)))
    
    return tangents

def generate_tangents_fixture_point(fixture, point):
    tangents = []
    x0 = fixture['x']
    y0 = fixture['y']
    r0 = fixture['dilated_radius"]']

    x1 = point['x']
    y1 = point['y']
    
    delta_x = x1-x0
    delta_y = y1-y0
    distance = np.sqrt(delta_x**2 + delta_y**2)
    # k=1
    for k in [-1,1]:
        c0_x = -(r0**2)/(distance**2)*delta_x - (k*r0/distance**2)*np.sqrt(distance**2-r0**2)*delta_y
        c0_y = -(r0**2)/(distance**2)*delta_y + (k*r0/distance**2)*np.sqrt(distance**2-r0**2)*delta_x
        
        tangents.append(((c0_x+x0, x1), (c0_y+y0, y1)))
    
    return tangents


def plot_board(ax, route_data, tangent_lines, start_points, end_points):
    plt.grid(True)
    for fixture_id, fixture in route_data["Fixtures"].items():
        if fixture["type"] == "CIRCLE":
            circle_array = create_circle(fixture["x"],fixture["y"],fixture["radius"])
            ax.plot(circle_array[0], circle_array[1], 'k')
            ax.text(fixture["x"], fixture["y"], str(fixture_id), fontsize=12, color="g")
        elif fixture["type"] == "POINT":
            ax.plot(fixture["x"], fixture["y"], 'ro')
            ax.text(fixture["x"], fixture["y"], str(fixture_id), fontsize=12, color="g")
        elif fixture["type"] == "CHANNEL":
            points = get_channel_points(fixture)
            ax.plot([points[0][0], points[1][0]], [points[0][1], points[1][1]], 'k')
            ax.plot([points[1][0], points[2][0]], [points[1][1], points[2][1]], 'k')
            ax.plot([points[2][0], points[3][0]], [points[2][1], points[3][1]], 'k')
            ax.plot([points[3][0], points[0][0]], [points[3][1], points[0][1]], 'k')
            ax.text(fixture["x"], fixture["y"], str(fixture_id), fontsize=12, color="g")
    # To PLOT:
    for tangent in tangent_lines:
        ax.plot([tangent[0][0], tangent[1][0]], [tangent[0][1], tangent[1][1]], 'k--')
        
    for point in end_points:
        ax.plot(point[0], point[1], "r*")

    for point in start_points:
        ax.plot(point[0], point[1], "bo")

def get_channel_points(channel: dict):
    x = channel["x"]
    y = channel["y"]
    rz = channel["rz"]
    width = channel["width"]
    height = channel["height"]

    p1 = np.array([-width/2,-height/2])
    p2 = np.array([-width/2, height/2])
    p3 = np.array([width/2 , height/2])
    p4 = np.array([width/2 ,-height/2])

    # Rotate the points by rz
    rot_matrix = np.array([[np.cos(rz), -np.sin(rz)], [np.sin(rz), np.cos(rz)]])
    p1_rot = np.dot(rot_matrix, p1)
    p2_rot = np.dot(rot_matrix, p2)
    p3_rot = np.dot(rot_matrix, p3)
    p4_rot = np.dot(rot_matrix, p4)

    # Translate the points by x,y
    transla = np.array([x,y])
    p1_r_t = p1_rot + transla
    p2_r_t = p2_rot + transla
    p3_r_t = p3_rot + transla
    p4_r_t = p4_rot + transla

    return (p1_r_t, p2_r_t, p3_r_t, p4_r_t)

def save_route_robot(end_points, output_path):
    out_dict = {"Points":[]}
    for point in end_points:
        out_dict["Points"].append({"x":point[0], "y":point[1]})
    # Serializing json
    json_object = json.dumps(out_dict, indent=4)
    
    with open(output_path, "w") as outfile:
        outfile.write(json_object)
    
    return True
