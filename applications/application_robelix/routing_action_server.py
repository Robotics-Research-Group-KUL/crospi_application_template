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

from geometry_msgs.msg import Pose

from std_srvs.srv import Empty
from skill_specifications.libraries.cable_routing_lib.skill_specifications.betfsm_channel_fixture_skill import ChannelFixtureSkill
from skill_specifications.libraries.cable_routing_lib.skill_specifications.betfsm_pivot_fixture_skill import PivotFixtureSkill, MoveGripperToPosition
from skill_specifications.libraries.peg_insertions_robelix.skill_specifications.peg_insertion_skill import PegInsertionSkill
from skill_specifications.libraries.cable_routing_lib.skill_specifications.betfsm_pickup_fixture_skill import PickupFixtureSkill


from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MavLCSM(TickingStateMachine):
    def __init__(self, name:str, mav_node:str, node:Node = None):

        super().__init__(name, outcomes=[SUCCEED, ABORT, TIMEOUT])

        self.add_state(LifeCycle(name="DeactivateMAV", srv_name=f"/{mav_node}", 
                                 transition=Transition.DEACTIVATE, node=node, 
                                 timeout=Duration(seconds=5)),
                                 transitions={SUCCEED: "CleanupMAV",
                                                ABORT: "CleanupMAV",
                                                TIMEOUT: ABORT})

        self.add_state(LifeCycle(name="CleanupMAV", srv_name=f"/{mav_node}", 
                                 transition=Transition.CLEANUP, node=node, 
                                 timeout=Duration(seconds=5)),
                                 transitions={SUCCEED: "ConfigureMAV",
                                                ABORT: "ConfigureMAV",
                                                TIMEOUT: ABORT})

        self.add_state(LifeCycle(name="ConfigureMAV", srv_name=f"/{mav_node}", 
                                 transition=Transition.CONFIGURE, node=node, 
                                 timeout=Duration(seconds=5)),
                                 transitions={SUCCEED: "ActivateMAV",
                                                ABORT: ABORT})

        self.add_state(LifeCycle(name="ActivateMAV", srv_name=f"/{mav_node}", 
                                 transition=Transition.ACTIVATE, node=node, 
                                 timeout=Duration(seconds=5)),
                                 transitions={SUCCEED: SUCCEED})


class ComputeFixturePosition(Generator):
    def __init__(self, fixture_id:int, bb_location:str):
        super().__init__("GetFixturePosition",[SUCCEED])
        self.fixture_id = fixture_id
        self.bb_location = bb_location
    
    def co_execute(self,blackboard):
        pos_x = blackboard["output_param"][self.bb_location]["x_tf"]
        pos_y = blackboard["output_param"][self.bb_location]["y_tf"]
        pos_z = blackboard["output_param"][self.bb_location]["z_tf"]

        T_root_board = blackboard["initial_board_pose"]

        T_root_fixture = np.eye(4)
        T_root_fixture[:3, 3] = np.array([pos_x, pos_y, pos_z])

        T_board_fixture = np.linalg.inv(T_root_board) @ T_root_fixture

        if "board_real" not in blackboard:
            blackboard["board_real"] = {}
        blackboard["board_real"][self.fixture_id] = T_board_fixture[:2, 3].tolist()

        yield SUCCEED

class GetFrame(Generator):
    def __init__(self, params, route_task_model, board_model):
        super().__init__("GetFrame",[SUCCEED])
        self.route_task_model = route_task_model
        self.board_model = board_model
        self.params = params

    def co_execute(self,bb):
        T_root_board = bb["initial_board_pose"]

        initial_fixture_id = self.route_task_model["Route"][0]
        initial_fixture = self.board_model["Fixtures"][initial_fixture_id]
        T_board_2_initial_fixture = np.eye(4)
        T_board_2_initial_fixture[:3, 3] = np.array([initial_fixture["x"]+ self.params["initial_fixture_offset"], initial_fixture["y"], self.params["initial_fixture_height"]])

        T_board_2_initial_fixture_offset = np.eye(4)
        T_board_2_initial_fixture_offset[:3, 3] = np.array([initial_fixture["x"]+ self.params["initial_fixture_offset"], initial_fixture["y"], self.params["initial_fixture_height"]+0.1])

        T_root_2_initial_fixture = T_root_board @ T_board_2_initial_fixture
        T_root_2_initial_fixture_offset = T_root_board @ T_board_2_initial_fixture_offset

        # Roation of the frame so the gripper arrives with the positive x-axis in the direction of the initial fixture
        T_board_gripper = np.eye(4)
        T_board_gripper[:3, 3] = np.array([0.0, 0.0, 0.0])
        T_board_gripper[:3, :3] = R.from_euler('ZYX', [np.pi, 0.0, 0.0], degrees=False).as_matrix()

        T_root_gripper = T_root_2_initial_fixture @ T_board_gripper
        T_root_gripper_offset = T_root_2_initial_fixture_offset @ T_board_gripper

        pos_root_gripper = T_root_gripper[:3, 3]
        rot_root_gripper = R.from_matrix(T_root_gripper[:3, :3]).as_quat()

        pos_root_gripper_offset = T_root_gripper_offset[:3, 3]

        bb["application_params"]["fixture_1_pose"] = [pos_root_gripper[0], pos_root_gripper[1], pos_root_gripper[2],
                                                      rot_root_gripper[0], rot_root_gripper[1], rot_root_gripper[2], rot_root_gripper[3]]
        
        bb["application_params"]["fixture_1_pose_offset"] = [pos_root_gripper_offset[0], pos_root_gripper_offset[1], pos_root_gripper_offset[2],
                                                      rot_root_gripper[0], rot_root_gripper[1], rot_root_gripper[2], rot_root_gripper[3]]
        
        yield SUCCEED

class SetupGoalResult(Generator):
    def __init__(self, result):
        super().__init__("SetupGoalResult",[SUCCEED])
        self.result = result

    def co_execute(self,blackboard):
        board_model_updated = {"Fixtures": {}}
        for fixture_id in blackboard["board_real"].keys():
            board_model_updated["Fixtures"][fixture_id] = {}
            board_model_updated["Fixtures"][fixture_id]["x"] = blackboard["board_real"][fixture_id][0]
            board_model_updated["Fixtures"][fixture_id]["y"] = blackboard["board_real"][fixture_id][1]

        self.result.outcome = "Success"
        self.result.parameters = json.dumps({'board': board_model_updated})

        yield SUCCEED

class PublishFeedback(Generator):
    def __init__(self, goal_handle, id:str):
        super().__init__("PublishFeedback",[SUCCEED])
        self.goal_handle = goal_handle
        self.id = id

    def co_execute(self,blackboard):
        feedback_msg = Task.Feedback()
        feedback_msg.state = self.id
        self.goal_handle.publish_feedback(feedback_msg)
        yield SUCCEED

class ReadLogFromTopic(Generator):
    def __init__(self, topic_name:str, bb_key:str, number_data:int, node: Node = None):
        super().__init__("ReadFromTopic",[SUCCEED])
        self.node = node
        self.average_msg = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.topic = topic_name
        self.bb_key = bb_key
        self.number_data = number_data
        self.actual_data = 0
        self.subscription = None

    def cb_msg(self,msg):
        euler = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]).as_euler('xyz', degrees=False)

        self.average_msg += np.array([msg.position.x, msg.position.y, msg.position.z, euler[0], euler[1], euler[2]])

        self.actual_data += 1

    def co_execute(self,blackboard):
        # subscribe to the topic
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # typical depth for sensor data
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=rclpy.duration.Duration(seconds=0),  # messages never expire
            deadline=rclpy.duration.Duration(seconds=0)  # no deadline
            # liveliness=rclpy.qos.LivelinessPolicy.SYSTEM_DEFAULT,
            # liveliness_lease_duration=rclpy.duration.Duration(seconds=0),
            # avoid_ros_namespace_conventions=False,
        )
        self.subscription = self.node.create_subscription(Pose, self.topic, self.cb_msg, sensor_qos)
        # store the message in the blackboard
        # TODO: Add a timeout
        while self.actual_data < self.number_data:
            yield TICKING
        
        self.average_message = self.average_msg / self.actual_data
        T_matrix = np.eye(4)
        T_matrix[:3, 3] = self.average_message[:3]
        T_matrix[:3, :3] = R.from_euler('xyz', self.average_message[3:6], degrees=False).as_matrix()
        blackboard[self.bb_key] = T_matrix

        quat = R.from_euler('xyz', self.average_message[3:6], degrees=False).as_quat()
        T_root_board = [self.average_message[0], self.average_message[1], self.average_message[2],
                        quat[0], quat[1], quat[2], quat[3]]
        blackboard["output_param"]["vision_detection"] = {}
        blackboard["output_param"]["vision_detection"][self.bb_key] = T_root_board
        get_logger().info(f"Read {self.actual_data} messages from topic {self.topic}, averaged to: ------------- \n {blackboard[self.bb_key]}")

        # Destroy the subscription properly
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
        yield SUCCEED


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
        self.load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/cable_routing_lib/tasks/pickup_fixture_skill.json")
        self.load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/peg_insertions_robelix/tasks/peg_insertion_skill.json")

        self.blackboard["application_params"] = self.params


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing cable routing task ...')
        self.get_logger().info(f"Received task: {goal_handle.request.task}")
        self.get_logger().info(f"Received route: {goal_handle.request.parameters}")

        result = Task.Result()

        parameters_str = goal_handle.request.parameters
        print("Parameters string: ", parameters_str)
        if goal_handle.request.task == "Routing_Task":
            print("Starting Routing_Task")
            board_model, route_task_model = self.get_board_and_route(parameters_str)

            routing_sm = self.generate_routing_sm(board_model, route_task_model)

            # WARNING: Do not change the home position of the robot or add a cartesian space motion before the taring to ensure z-axis is aligned with gravity
            sm_to_execute = Sequence("Intial", children = [
                                                            # LifeCycle(name="ConfigureMAV", srv_name="/schwarzmuller_driver_lifecycle_node", transition=Transition.CONFIGURE, node=self, timeout=Duration(seconds=5)),
                                                            # LifeCycle(name="ActivateMAV", srv_name="/schwarzmuller_driver_lifecycle_node", transition=Transition.ACTIVATE, node=self, timeout=Duration(seconds=5)),
                                                            MavLCSM(name="MAV_LCSM", mav_node="schwarzmuller_driver_lifecycle_node", node=self),
                                                            # eTaSL_StateMachine("MovingHome", "MovingHome", node=self),
                                                            eTaSL_StateMachine("MovingHome_withMav", "MovingHome_withMav", node=self),
                                                            MoveGripperToPosition(finger_position=0.0, gripping_velocity=self.params["gripper_vel"], node=self),
                                                            ServiceClient("taring_ft_sensor","/schunk/tare_sensor", Empty, [SUCCEED], node=self),
                                                            TimedWait("timed_wait",Duration(seconds=1.0),node=self ),
                                                            ReadLogFromTopic('/charuco_detector/pose', 'initial_board_pose', 10, node=self),
                                                            GetFrame(params=self.params, board_model=board_model, route_task_model=route_task_model),
                                                            eTaSL_StateMachine("goingToPreStarting", "GoingToPreStarting", node=self),
                                                            eTaSL_StateMachine("goingToStarting", "GoingToStarting", node=self),
                                                            routing_sm])
            
        elif goal_handle.request.task == "Prepare_board":
            print("Prepare_board task")
            board_model = self.get_board(parameters_str)
            sm_to_execute = self.generate_board_assembly_sm(board_model=board_model, result=result, goal_handle=goal_handle)


        rate = self.create_rate(100)
        rate.sleep()

        sm_to_execute.reset()
        outcome=sm_to_execute(self.blackboard)
        while outcome==TICKING:
            rate.sleep()
            outcome=sm_to_execute(self.blackboard)
        self.get_logger().info(f'Finished state machine {goal_handle.request.task} with outcome {outcome} and result: {result.parameters}')

        print("Final board positions: ", self.blackboard["board_real"])
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
    
    def get_board(self, parameters_str):
        """
        Extract the board model task model from the parameters string.
        :param parameters_str: The parameters string containing the board information.
        :return: A tuple containing the board model
        """

        parameters_dict = json.loads(parameters_str)
        board_model = parameters_dict["board"]

        for fixture_id in board_model["Fixtures"]:
            fixture = board_model["Fixtures"][fixture_id]
            if fixture["type"] == "CHANNEL":
                fixture["width"] = self.params["channel_width"]
                fixture["height"] = self.params["channel_height"]
                fixture["length"] = self.params["channel_length"]
            elif fixture["type"] == "PIVOT":
                fixture["radius"] = self.params["pivot_radius"]
                fixture["dilated_radius"] = self.params["pivot_dilated_radius"]

        return board_model

    def get_board_and_route(self, parameters_str):
        """
        Extract the route task model from the parameters string.
        :param parameters_str: The parameters string containing the route information.
        :return: A tuple containing the route task model.
        """
        parameters_dict = json.loads(parameters_str)
        route_task_model = {}
        route_task_model["Route"] = list(parameters_dict["route"].keys())
        board_model = {"Fixtures": parameters_dict["route"]}
        for fixture_id in board_model["Fixtures"]:
            fixture = board_model["Fixtures"][fixture_id]
            if fixture["type"] == "CHANNEL":
                fixture["width"] = self.params["channel_width"]
                fixture["height"] = self.params["channel_height"]
                fixture["length"] = self.params["channel_length"]
                fixture["rz"] = 0.0
            elif fixture["type"] == "PIVOT":
                fixture["radius"] = self.params["pivot_radius"]
                fixture["dilated_radius"] = self.params["pivot_dilated_radius"]

        return board_model, route_task_model
    
    def generate_board_assembly_sm(self, board_model, result, goal_handle):
        board_assembly_sm = []
        # board_assembly_sm.append(LifeCycle(name="ConfigureMAV", srv_name="/schwarzmuller_driver_lifecycle_node", transition=Transition.CONFIGURE, node=self, timeout=Duration(seconds=5)))
        # board_assembly_sm.append(LifeCycle(name="ActivateMAV", srv_name="/schwarzmuller_driver_lifecycle_node", transition=Transition.ACTIVATE, node=self, timeout=Duration(seconds=5)))
        board_assembly_sm.append(MavLCSM(name="MAV_LCSM", mav_node="schwarzmuller_driver_lifecycle_node", node=self))
        board_assembly_sm.append(eTaSL_StateMachine("MovingHome", "MovingHome", node=self))
        # TODO: Add cartesian place to tare the FT sensor
        board_assembly_sm.append(ServiceClient("taring_ft_sensor","/schunk/tare_sensor", Empty, [SUCCEED], node=self))
        board_assembly_sm.append(ReadLogFromTopic('/charuco_detector/pose', 'initial_board_pose', 10, node=self))
        fixture_poses = []
        for i, fixture_id in enumerate(board_model["Fixtures"].keys()):
            skill_params = {}
            if board_model["Fixtures"][fixture_id]["y"] >= 0.46:
                skill_params["fixture_x_coordinate"] = board_model["Fixtures"][fixture_id]["x"] + 0.05
            else:
                skill_params["fixture_x_coordinate"] = board_model["Fixtures"][fixture_id]["x"] + 0.15
            fixture = board_model["Fixtures"][fixture_id]
            if fixture["type"] == "CHANNEL":
                print("Adding pickup skill for channel fixture")
                board_assembly_sm.append(PickupFixtureSkill(node=self, fixture_type="CHANNEL", skill_params=skill_params))
                print("Pickup skill added for channel fixture")
            elif fixture["type"] == "PIVOT":
                print("Adding pickup skill for pivot fixture")
                board_assembly_sm.append(PickupFixtureSkill(node=self, fixture_type="PIVOT", skill_params=skill_params))
                print("Pickup skill added for pivot fixture")

            fixture_poses.append([fixture["x"], fixture["y"], 0.05,
                            0, 0, 0, 1])
            board_assembly_sm.append(eTaSL_StateMachine(f"MovingHome_{fixture_id}", "MovingHome", node=self))
            board_assembly_sm.append(ReadLogFromTopic('/charuco_detector/pose', 'initial_board_pose', 10, node=self))
            board_assembly_sm.append(ServiceClient("taring_ft_sensor","/schunk/tare_sensor", Empty, [SUCCEED], node=self))
            board_assembly_sm.append(eTaSL_StateMachine(f"goingOnTopFixture_{fixture_id}", "GoingOnTopFixture", 
                                                        cb = lambda bb, i=i: {
                                                            "desired_pose": fixture_poses[i]
                                                        }, 
                                                        node=self))
            
            # Append insertion skill
            board_assembly_sm.append(PegInsertionSkill(node=self, id=fixture_id, skill_params={}))
            board_assembly_sm.append(ComputeFixturePosition(fixture_id=fixture_id, bb_location=f"Rotate90z{fixture_id}"))
            # TO CHECK ROTATION DIRECTION IN SIMULATION UNCOMMENT THE FOLLOWING LINE
            # # # board_assembly_sm.append(ComputeFixturePosition(fixture_id=fixture_id, bb_location=f"Rotate90z_simulation{fixture_id}"))
            board_assembly_sm.append(LogBlackboard("Logblackboard2",["output_param"]))
            board_assembly_sm.append(PublishFeedback(goal_handle=goal_handle, id=fixture_id))
            board_assembly_sm.append(eTaSL_StateMachine(f"MovingHomeFinishing_{fixture_id}", "MovingHome", node=self))


        board_assembly_sm.append(SetupGoalResult(result))
            # board_assembly_sm.append(TimedWait(f"TimedWait_{fixture_id}", Duration(seconds=10.0), node=self))
        print("Board assembly state machine: ", board_assembly_sm)
        return Sequence("Intial", children = board_assembly_sm)


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
        prev_segment_dir = -1
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

            current_fixture_wrt_start_fixture = current_fixture_frame @ np.linalg.inv(start_fixture_frame)

            skill_params["turning_dir_sliding"] = direction_segment
            skill_params["desired_pos"] = [waypoint_x, waypoint_y, waypoint_z]
            skill_params["gripper_vel"] = self.params["gripper_vel"]
            skill_params["gripper_force"] = self.params["gripper_force"]
            skill_params["gripper_direction"] = self.params["gripper_direction"]
            skill_params["cable_slide_pos"] = self.params["cable_slide_pos"]
            
            if current_fixture["type"] == "PIVOT":
                # pivoting_dir
                skill_params["turning_dir_pivoting"] = -direction_segment
                skill_params["previous_to_current_fixture"] = [current_fixture_wrt_start_fixture[0,3], current_fixture_wrt_start_fixture[1,3], 
                                                                current_fixture_wrt_start_fixture[2,3]]
                
                skill_params["z_down"] = self.params["pivot_z_down"]

                skill_params["frame_next_fixture_wrt_board"] = [next_fixture["x"], next_fixture["y"], self.params["slide_height"],
                                                   0, 0, 0, 1]

                # print(f"Fixture {current_fixture_id} parameters: {skill_params}")
                
                routing_sm.append(PivotFixtureSkill(node=self, id=current_fixture_id, skill_params=skill_params))

            if current_fixture["type"] == "CHANNEL":
                T_insertion_frame = np.eye(4)
                T_insertion_frame[:3, 3] = np.array([current_fixture["x"], current_fixture["y"], self.params["slide_height"]])
                
                T_offset = np.eye(4)
                T_offset[:3, 3] = np.array([self.params["channel_width"] + self.params["gripper_width"], 0.0, 0.0])
                
                T_rotation = np.eye(4)
                T_rotation[:3, :3] = R.from_euler('z', channel_rz).as_matrix()

                T_channel_frame = T_insertion_frame @ T_rotation @ T_offset
                channel_frame_x, channel_frame_y, channel_frame_z = T_channel_frame[:3, 3]
                
                Rot_channel = R.from_euler('z', channel_rz).as_matrix() @ R.from_euler('ZXY', [np.pi, 0, 0]).as_matrix()
                quaternion = R.from_matrix(Rot_channel).as_quat()
                skill_params["channel_aligning_pose"] = [channel_frame_x, channel_frame_y, self.params["channel_height"],
                                                            quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
                skill_params["channel_inserting_pose"] = [channel_frame_x, channel_frame_y, self.params["channel_height"] - self.params["channel_insertion_diff"],
                                                            quaternion[0], quaternion[1], quaternion[2], quaternion[3]]

                # print(f"Fixture {current_fixture_id} parameters: {skill_params}")
                
                routing_sm.append(ChannelFixtureSkill(node=self, id=current_fixture_id, skill_params=skill_params))
        
        # ---------------------------- Last fixture --------------------------------------------------------
        last_fixture_id = route_task_model["Route"][-1]
        prev_fixture_id = route_task_model["Route"][-2]

        last_fixture = board_model["Fixtures"][last_fixture_id]
        prev_fixture = board_model["Fixtures"][prev_fixture_id]


        if last_fixture["type"] == "CHANNEL":
            skill_params = {}
            direction_segment = prev_segment_dir
            route_task_model["Directions"].append(direction_segment)

            tangent = ru.compute_tangent(prev_fixture, last_fixture, d1 = prev_segment_dir, d2 = direction_segment, dilate_2 = True)
            
            tangent_lines.append(tangent)
            # pdb.set_trace()
            unit_tangent_vector = np.array(np.array(tangent[1]) - np.array(tangent[0]))/np.linalg.norm(np.array(tangent[1]) - np.array(tangent[0]))

            # Modify the slack for channels based on the orientation difference between the tangent vector and the channel fixture orientation
            
            orientation_1 = last_fixture["rz"]
            orientation_2 = last_fixture["rz"] + np.pi

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

            skill_params["turning_dir_sliding"] = direction_segment
            skill_params["desired_pos"] = [waypoint_x, waypoint_y, waypoint_z]
            skill_params["gripper_vel"] = self.params["gripper_vel"]
            skill_params["gripper_force"] = self.params["gripper_force"]
            skill_params["gripper_direction"] = self.params["gripper_direction"]
            skill_params["cable_slide_pos"] = self.params["cable_slide_pos"]

            T_insertion_frame = np.eye(4)
            T_insertion_frame[:3, 3] = np.array([last_fixture["x"], last_fixture["y"], self.params["slide_height"]])
            
            T_offset = np.eye(4)
            T_offset[:3, 3] = np.array([self.params["channel_width"] + self.params["gripper_width"], 0.0, 0.0])
            
            T_rotation = np.eye(4)
            T_rotation[:3, :3] = R.from_euler('z', channel_rz).as_matrix()

            T_channel_frame = T_insertion_frame @ T_rotation @ T_offset
            channel_frame_x, channel_frame_y, channel_frame_z = T_channel_frame[:3, 3]
            
            Rot_channel = R.from_euler('z', channel_rz).as_matrix() @ R.from_euler('ZXY', [np.pi, 0, 0]).as_matrix()
            quaternion = R.from_matrix(Rot_channel).as_quat()
            skill_params["channel_aligning_pose"] = [channel_frame_x, channel_frame_y, self.params["channel_height"],
                                                        quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
            skill_params["channel_inserting_pose"] = [channel_frame_x, channel_frame_y, self.params["channel_height"] - self.params["channel_insertion_diff"],
                                                        quaternion[0], quaternion[1], quaternion[2], quaternion[3]]

            # print(f"Fixture {last_fixture_id} parameters: {skill_params}")
            
            routing_sm.append(ChannelFixtureSkill(node=self, id=last_fixture_id, skill_params=skill_params))

        # -----------------------------------------------------------------------------------------------------
            
        print("Directions: ", route_task_model["Directions"])
        print("SM: ", routing_sm)
        print("Board: ", board_model)
        fig, ax = plt.subplots()
        ru.plot_board(ax, board_model,tangent_lines,[],waypoints)
        ax.set_ylim(0.0, 0.91)
        plt.axis('equal')
        # save the figure
        plt.savefig("waypoints.pdf")
        
        # import pdb
        # pdb.set_trace()

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