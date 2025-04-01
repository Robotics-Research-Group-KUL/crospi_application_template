import rclpy
import sys

from betfsm.betfsm import *
from betfsm.betfsm_ros import *
from betfsm.betfsm_etasl import *
from betfsm.betfsm_action_server import *

import math

from std_srvs.srv import Empty
from geometry_msgs.msg import Wrench

from dualquaternion import quat
from dualquaternion import SO3, SE3

import numpy as np
import pdb
import os
sm_path = os.path.dirname(os.path.abspath(__file__))
from rclpy.qos import QoSProfile
import json
from schunk_control_egh80.srv import ControlEGH80
from pickit_comm import PickitInterface

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

class ConfigurePickit(Generator):
    """
    Configure the Pickit interface with the given parameters and store it in the blackboard under the given key.
    """
    def __init__(self, ip:str, port: int, setup_id: int, product_id: int, mult: int, bb_key_interface:str):
        super().__init__("ConfigurePickit",[SUCCEED, CANCEL])
        self.ip = ip
        self.port = port
        self.setup_id = setup_id
        self.product_id = product_id
        self.mult = mult
        self.bb_key_interface = bb_key_interface

    def co_execute(self,blackboard):
        self.pickitI = PickitInterface(self.ip, self.port, self.mult, non_blocking=True)
        blackboard[self.bb_key_interface] = self.pickitI
        yield TICKING
        self.pickitI.configure_environment(self.setup_id, self.product_id)
        while not self.pickitI.is_response_ready():
            yield TICKING
        succeed = self.pickitI.get_configure_environment_result()
        if succeed:
            print("[Pickit] Environment Configured")
            yield SUCCEED
        else:
            print("[Pickit] Error configuring environment")
            yield CANCEL

class CaptureImage(Generator):
    """
    Capture an image from the Pickit interface.
    """
    def __init__(self, bb_key_interface:str):
        super().__init__("CaptureAndProcess",[SUCCEED, CANCEL])
        self.bb_key_interface = bb_key_interface

    def co_execute(self,blackboard):
        pickitI = blackboard[self.bb_key_interface]
        pickitI.capture_image()
        while not pickitI.is_response_ready():
            yield TICKING
        succeed = pickitI.get_capture_image_result()
        if succeed:
            print("[Pickit] Imaged caputured")
            yield SUCCEED
        else:
            print("[Pickit] Error capturing image")
            yield CANCEL

class DetectObjects(Generator):
    """
    Detect objects in the last image captured by the Pickit interface.
    """
    def __init__(self, bb_key_object:str, bb_key_interface:str):
        super().__init__("DetectObjects",[SUCCEED, CANCEL])
        self.bb_key_object = bb_key_object
        self.bb_key_interface = bb_key_interface

    def co_execute(self,blackboard):
        pickitI = blackboard[self.bb_key_interface]
        pickitI.process_image()
        while not pickitI.is_response_ready():
            yield TICKING
        response = pickitI.get_process_image_result()
        if response["object_found"]:
            blackboard[self.bb_key_object] = response
            print("Object Detected: ", response)
            yield SUCCEED
        else:
            print("No object detected")
            yield CANCEL
    

class ChangePickitSetup(Generator):
    """
    Change the Pickit setup and product to the given setup_id and product_id respectively.
    """
    def __init__(self, bb_key_interface:str, setup_id: int, product_id: int):
        super().__init__("ChangePickitSetup",[SUCCEED])
        self.bb_key_interface = bb_key_interface
        self.setup_id = setup_id
        self.product_id = product_id

    def co_execute(self,blackboard):
        pickitI = blackboard[self.bb_key_interface]
        pickitI.configure_environment(self.setup_id, self.product_id)
        while not pickitI.is_response_ready():
            yield TICKING
        succeed = pickitI.get_configure_environment_result()
        if succeed:
            print("[Pickit] Environment Configured")
            yield SUCCEED
        else:
            print("[Pickit] Error configuring environment")
            yield CANCEL
   
def get_objects_pose(pickit_ip, pickit_port, pickit_setup_id, pickit_object_dict, pickit_mult, node=None):
    """
    Sequence of tasks to get the pose of the object in the Pickit interface.
    pickit_object_dict: dictionary with the object name as key and the object id in the Pickit interface as value.
    """
    configured_and_image_captured = False
    state_list = []
    for object_key, object_id in pickit_object_dict.items():
        if not configured_and_image_captured:
            state_list.append(ConfigurePickit(pickit_ip, pickit_port, pickit_setup_id, object_id, pickit_mult, "pickit_interface"))
            state_list.append(CaptureImage("pickit_interface"))
            configured_and_image_captured = True
        else:
            state_list.append(ChangePickitSetup("pickit_interface", pickit_setup_id, object_id))
        state_list.append(DetectObjects(object_key, "pickit_interface"))
    return Sequence("ConfigureCaptureandDetect", children=state_list)

def insertion_FSM(node=None):

    return None

class GripperClient(ServiceClient):
    def __init__(self, name:str, request_data:Dict, 
                 timeout:Duration = Duration(seconds=5.0), node:Node = None):
        super().__init__(name=name, srv_name="/schunk/control/schunk_egh80", srv_type=ControlEGH80,
                         outcomes=[SUCCEED],timeout=timeout, node=node)
        self.request_data = request_data

    def fill_in_request(self, blackboard, request):
        request.command = self.request_data.get('command',0)
        request.position = self.request_data.get('position', 0.0)
        return request

class ComputeFrames(Generator):
    def __init__(self, bb_key_PCB:str, bb_key_fixture:str):
        super().__init__("ComputeFrames",[SUCCEED, CANCEL])
        self.bb_key_PCB = bb_key_PCB
        self.bb_key_fixture = bb_key_fixture
    
    def co_execute(self,blackboard):
        blackboard["output_param"] = {}
        blackboard["output_param"]["ComputeFrames"] = {}

        # Params
        pcb_lifting = 0.07 # m
        offset_x = -0.008
        offset_y = -0.008
        offset_z = 0.005
        offset_rx = -20*np.pi/180
        offset_ry = 20*np.pi/180
        offset_rz = 5*np.pi/180

        # Frames from perception
        pcb_frame = blackboard[self.bb_key_PCB]
        fixture_frame = blackboard[self.bb_key_fixture]
        pcb_wrt_world = SE3.frame(SO3.from_quat(pcb_frame["orientation"]), pcb_frame["position"])
        fixture_wrt_world = SE3.frame(SO3.from_quat(fixture_frame["orientation"]), fixture_frame["position"])
        blackboard["output_param"]["ComputeFrames"]["fixture_wrt_world"] = [fixture_frame["position"][0], fixture_frame["position"][1], fixture_frame["position"][2],
                                                                fixture_frame["orientation"][1], fixture_frame["orientation"][2], fixture_frame["orientation"][3], fixture_frame["orientation"][0]]
                                                                            

        # Picking frame w.r.t. the pcb frame
        picking_frame_pcb_R = SO3.RPY(-0.5*np.pi/180,-2*np.pi/180,-np.pi/2)
        # quaternion_picking_frame_pcb = quat.from_R(picking_frame_pcb_R)
        picking_frame_pcb_x = 0.0275
        picking_frame_pcb_y = 0.045
        picking_frame_pcb_z = -0.003
        picking_frame_pcb = SE3.frame(picking_frame_pcb_R, np.array([picking_frame_pcb_x, picking_frame_pcb_y, picking_frame_pcb_z]))

        picking_frame_wrt_world = np.dot(pcb_wrt_world, picking_frame_pcb)
        picking_world_p = SE3.origin(picking_frame_wrt_world)
        quaternion_picking_world = quat.from_R(SE3.orient(picking_frame_wrt_world))
        blackboard["output_param"]["ComputeFrames"]["pcb_picking_pose"] = [picking_world_p[0], picking_world_p[1], picking_world_p[2], 
                                                           quaternion_picking_world[1], quaternion_picking_world[2], quaternion_picking_world[3], quaternion_picking_world[0]]
        blackboard["output_param"]["ComputeFrames"]["pcb_lifting_pose"] = [picking_world_p[0], picking_world_p[1], picking_world_p[2]+pcb_lifting, 
                                                           quaternion_picking_world[1], quaternion_picking_world[2], quaternion_picking_world[3], quaternion_picking_world[0]]
        
        # Fixture frame w.r.t. tcp frame
        fixture_insertion_frame_wrt_fixture_frame = SE3.frame(SO3.rotate_z(-np.pi/2), np.array([0.0275,0.045,0.0]))

        fixture_frame_top = np.dot(fixture_wrt_world, fixture_insertion_frame_wrt_fixture_frame)
        fixture_world_p = np.copy(SE3.origin(fixture_frame_top))
        quaternion_fixture_world = quat.from_R(SE3.orient(fixture_frame_top))
        fixture_world_p[2] = fixture_world_p[2] + pcb_lifting

        blackboard["output_param"]["ComputeFrames"]["fixture_insertion_starting_pose"] = [fixture_world_p[0], fixture_world_p[1], fixture_world_p[2],
                                                              quaternion_fixture_world[1], quaternion_fixture_world[2], quaternion_fixture_world[3], quaternion_fixture_world[0]]
        
        # Coordinates of the corner of the PCB w.r.t. the picking frame
        PCB_corner_wrt_PCB_frame = SE3.frame(SO3.RPY(0,0,0), np.array([0.086,0.141,0.0]))
        PCB_frame_wrt_world_at_insertion_start = np.dot(fixture_frame_top, SE3.inv(picking_frame_pcb))
        PCB_corner_wrt_world = np.dot(PCB_frame_wrt_world_at_insertion_start, PCB_corner_wrt_PCB_frame)

        # PCB_corner_wrt_world = np.dot(fixture_frame_top, picking_frame_pcb)
        offset_orientation = np.dot(SO3.rotate_x(offset_rx), np.dot(SO3.rotate_z(offset_rz), SO3.rotate_y(offset_ry)))
        # SO3.RPY(offset_rx, offset_ry, offset_rz)
        PCB_corner_offset = SE3.frame(offset_orientation, np.array([offset_x,offset_y,offset_z]))
        offset_tcp_frame = np.dot(np.dot(np.dot(PCB_corner_wrt_world, PCB_corner_offset), SE3.inv(PCB_corner_wrt_PCB_frame)),picking_frame_pcb)
        offset_tcp_frame_p = SE3.origin(offset_tcp_frame)
        quaternion_offset_tcp_frame = quat.from_R(SE3.orient(offset_tcp_frame))
        
        blackboard["output_param"]["ComputeFrames"]["tcp_offset_pose"] = [offset_tcp_frame_p[0], offset_tcp_frame_p[1], offset_tcp_frame_p[2],
                                                                quaternion_offset_tcp_frame[1], quaternion_offset_tcp_frame[2], quaternion_offset_tcp_frame[3], quaternion_offset_tcp_frame[0]]
        
        tcp_frame_2_corner = np.dot(SE3.inv(picking_frame_pcb), PCB_corner_wrt_PCB_frame)
        tcp_frame_2_corner_p = SE3.origin(tcp_frame_2_corner)
        quaternion_tcp_frame_2_corner = quat.from_R(SE3.orient(tcp_frame_2_corner))

        blackboard["output_param"]["ComputeFrames"]["tcp_corner_pose"] = [tcp_frame_2_corner_p[0], tcp_frame_2_corner_p[1], tcp_frame_2_corner_p[2],
                                                                quaternion_tcp_frame_2_corner[1], quaternion_tcp_frame_2_corner[2], quaternion_tcp_frame_2_corner[3], quaternion_tcp_frame_2_corner[0]]
        

        tcp_frame_2_corner_oriented = np.dot(tcp_frame_2_corner, SE3.frame(np.linalg.inv(SE3.orient(PCB_corner_offset)), np.array([0,0,0])))
        tcp_frame_2_corner_oriented_p = SE3.origin(tcp_frame_2_corner_oriented)
        quaternion_tcp_frame_2_corner_oriented = quat.from_R(SE3.orient(tcp_frame_2_corner_oriented))

        blackboard["output_param"]["ComputeFrames"]["tcp_corner_oriented_pose"] = [tcp_frame_2_corner_oriented_p[0], tcp_frame_2_corner_oriented_p[1], tcp_frame_2_corner_oriented_p[2],
                                                                quaternion_tcp_frame_2_corner_oriented[1], quaternion_tcp_frame_2_corner_oriented[2], quaternion_tcp_frame_2_corner_oriented[3], quaternion_tcp_frame_2_corner_oriented[0]]
        
        yield SUCCEED

class ComputeFixtureCorner(Generator):
    def __init__(self):
        super().__init__("ComputeFixtureCorner",[SUCCEED])

    def co_execute(self,blackboard):
        # TODO: Check this is the correct frame and add the corresponding values to go to the upper corner
        x_coord = blackboard["output_param"]["Reducing_y_offset"]["task_frame_x"]
        y_coord = blackboard["output_param"]["Reducing_y_offset"]["task_frame_y"]
        z_coord = blackboard["output_param"]["Reducing_y_offset"]["task_frame_z"]
        frame_orientation = SO3.from_quat( np.array([blackboard["output_param"]["ComputeFrames"]["fixture_wrt_world"][6],
                                                     blackboard["output_param"]["ComputeFrames"]["fixture_wrt_world"][3],
                                                     blackboard["output_param"]["ComputeFrames"]["fixture_wrt_world"][4],
                                                     blackboard["output_param"]["ComputeFrames"]["fixture_wrt_world"][5]]))
            
        fixture_dx = 0.003
        fixture_dy = 0.003
        fixture_dz = 0.004
        tcp_frame_at_corner = SE3.frame(frame_orientation, np.array([x_coord, y_coord, z_coord]))
        fixture_corner = np.dot(tcp_frame_at_corner, SE3.frame(SO3.RPY(0,0,0), np.array([fixture_dx, fixture_dy, fixture_dz])))

        fixture_corner_oriented_offsets_p = SE3.origin(fixture_corner)
        blackboard["output_param"]["ComputeFrames"]["fixture_corner_oriented_pose"] = [fixture_corner_oriented_offsets_p[0], fixture_corner_oriented_offsets_p[1], fixture_corner_oriented_offsets_p[2],
                                                                blackboard["output_param"]["ComputeFrames"]["tcp_offset_pose"][3], blackboard["output_param"]["ComputeFrames"]["tcp_offset_pose"][4], blackboard["output_param"]["ComputeFrames"]["tcp_offset_pose"][5], blackboard["output_param"]["ComputeFrames"]["tcp_offset_pose"][6]]

        # pdb.set_trace()
        
        yield SUCCEED

def preInsert_FSM(sm, node=None):
    """
    Pre Insert FSM
    """

    open_gripper_data = {"command": ControlEGH80.Request.SET_POSITION,"position": 100.0}
    close_gripper_data = {"command": ControlEGH80.Request.SET_POSITION,"position": 0.0}

    open_gripper = GripperClient("OpenGripper", open_gripper_data, node=node)
    close_gripper = GripperClient("CloseGripper", close_gripper_data, node=node)

    tare_force_sensor = ServiceClient("taring_ft_sensor","/schunk/tare_sensor",Empty,[SUCCEED],node=node)
    concurrrentState = ConcurrentSequence("ConfigureWhileDetect", [open_gripper, sm, tare_force_sensor])

    preInsertSequence = Sequence("PreInsertion", children=[eTaSL_StateMachine("MovingHome","MovingHome",node=node),
                               concurrrentState,
                               eTaSL_StateMachine("MovingToPickingHome","MovingToPickingHome",node=node),
                               ComputeFrames("PCB","Fixture"),
                               eTaSL_StateMachine("MovingToPickPCB","MovingToPickPCB",node=node),
                               GripperClient("CloseGripper", close_gripper_data, node=node),
                               TimedWait("WaitingForGripper", Duration(seconds=2.0), node=node),
                               eTaSL_StateMachine("LiftingPCB","LiftingPCB",node=node),
                               eTaSL_StateMachine("MovingToPreInsert","MovingToPreInsert",node=node)])
    return preInsertSequence

def insert_FSM(node=None):
    """
    Insert FSM
    """
    tare_force_sensor_pre = ServiceClient("taring_ft_sensor_insertion","/schunk/tare_sensor",Empty,[SUCCEED],node=node)

    insertSequence = Sequence("Insertion", children=[tare_force_sensor_pre,
                                                     TimedWait("WaitAfterTaring", Duration(seconds=2.0), node=node),
                                                     eTaSL_StateMachine("MovingToOffsetPose","MovingToOffsetPose",node=node),
                                                     eTaSL_StateMachine("Reducing_z_offset","Reducing_z_offset",node=node),
                                                     eTaSL_StateMachine("Reducing_x_offset","Reducing_x_offset",node=node),
                                                     eTaSL_StateMachine("Reducing_y_offset","Reducing_y_offset", output_topic="/fsm_output_topic", node=node),
                                                     ComputeFixtureCorner(),
                                                     eTaSL_StateMachine("MovingToOffsetPose","MovingToOffsetPose",node=node),
                                                     eTaSL_StateMachine("MovingToFixtureCorner","MovingToFixtureCorner",node=node),
                                                     eTaSL_StateMachine("Reducing_ry_offset","Reducing_ry_offset",node=node),
                                                     eTaSL_StateMachine("Reducing_rz_offset","Reducing_rz_offset",node=node)])
    return insertSequence

def run_while_publishing(sm):
    """
    small state machine that executes `sm` while publishing graphviz to a topic /gz
    """

    #do_not_expand_types doesn't work for now...
    return Concurrent("concurrent",[
            sm,
            GraphvizPublisher("publisher","/gz",sm,None,skip=10,do_not_expand_types=[eTaSL_StateMachine])
    ])

def main(args=None):
    pickit_mult = 10000  # Scale factor for position and orientation values
    pickit_ip = "169.254.5.180"
    pickit_port = 5001
    pickit_setup_id = 24
    pickit_PCB_id = 46
    pickit_fixture_id = 47
    pickit_object_dict = {"PCB": pickit_PCB_id, "Fixture": pickit_fixture_id}

    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("pcb_insertion_skill")

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    #set_logger("state",my_node.get_logger())

    blackboard = {}

    load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/pcb_insertion_lib/tasks/pcb_insertion_skill.json", blackboard)

    sm_objects = get_objects_pose(pickit_ip, pickit_port, pickit_setup_id, pickit_object_dict, pickit_mult, my_node)
    sm_PCB_Insertion = Sequence("PCB_Insertion", children=[preInsert_FSM(sm_objects, my_node), insert_FSM(my_node)])
    sm = run_while_publishing(sm_PCB_Insertion)
    # prints a graphviz representation of sm:
    vis = GraphViz_Visitor()
    sm.accept(vis)
    vis.print()
    
    runner = BeTFSMRunner(my_node,sm,blackboard,0.01)

    rclpy.spin(my_node)
    
    # try:
    #     while (runner.get_outcome()==TICKING):
    #         rclpy.spin_once(my_node)
    #     rclpy.shutdown()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     print("final outcome : ",runner.get_outcome())
        
    print("shutdown")

if __name__ == "__main__":
    sys.exit(main(sys.argv))