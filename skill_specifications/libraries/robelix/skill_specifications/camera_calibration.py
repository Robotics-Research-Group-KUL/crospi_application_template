#!/usr/bin/env python3

import rclpy
import sys

from threading import Lock
import rclpy.time
from rclpy.node import Node

# from betfsm.betfsm import TickingState,TICKING,Blackboard, Sequence, Message, TickingStateMachine
from betfsm.betfsm import *
from betfsm.betfsm_node import BeTFSMNode
from betfsm.betfsm_etasl import load_task_list, eTaSL_StateMachine
from betfsm.graphviz_visitor import GraphViz_Visitor
from betfsm.logger import get_logger,set_logger
from betfsm.betfsm_ros import *


from rclpy.duration import Duration

import tf2_ros
# import PyKDL
# from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import TransformStamped
import numpy as np
import casadi as cas         




from tf_transformations import quaternion_matrix, translation_matrix


#from betfsm.graphviz_visitor import *


from betfsm.betfsm_action_server import CheckForCanceledAction

import math

# from skill_requirements import ParamValidator as pv

# param = pv.parameters("Task description goes here", [
#     pv.p_scalar({"name": "maxvel", "description": "Maximum velocity [m/s]", "default": 0.1, "required": True, "maximum": 0.5}),
#     pv.p_scalar({"name": "maxacc", "description": "Maximum acceleration [m/s^2]", "default": 0.1, "required": True, "maximum": 0.5}),
#     pv.p_scalar({"name": "eq_r", "description": "Equivalent radius", "default": 0.08, "required": False}),
#     pv.p_string({"name": "task_frame", "description": "Name of frame used to control the robot in cartesian space", "default": "tcp_frame", "required": False}),
#     pv.p_array({"name": "delta_pos", "type": "number", "default": [0.0, 0.0, 0.0], "description": "3D array of distances [m] that the robot will move w.r.t. the starting position in the X,Y,Z coordinates", "required": True, "minimum": -1.5, "maximum": 1.5, "minItems": 3, "maxItems": 3}),
# ])

class StorePose(Generator):
    def __init__(self):
        super().__init__("StorePose",[SUCCEED])    
        self.etasl_node = BeTFSMNode.get_instance()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.etasl_node)

    def co_execute(self,blackboard):


        if 'calibration_poses_robot' not in blackboard:
            blackboard['calibration_poses_robot'] = []

        if 'calibration_poses_camera' not in blackboard:
            blackboard['calibration_poses_camera'] = []


        # Append to blackboard
        blackboard['calibration_poses_robot'].append(self.get_and_store_tf(blackboard, target_frame = "tool0", source_frame = "world"))
        blackboard['calibration_poses_camera'].append(self.get_and_store_tf(blackboard, target_frame = "camera_frame", source_frame = "object_tracker_frame"))
        self.etasl_node.get_logger().info("TF stored in blackboard.")


        # pdb.set_trace()
        
        yield SUCCEED

    def get_and_store_tf(self, blackboard, target_frame: str, source_frame: str):
        try:
            # Lookup transform (latest available)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Convert to PyKDL.Frame
            t = transform.transform.translation
            q = transform.transform.rotation
            # translation = PyKDL.Vector(t.x, t.y, t.z)
            # rotation = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
            # frame = PyKDL.Frame(rotation, translation)

            print("------------------Transform:-------------------")
            print(transform.transform)

            # Extract translation and rotation (quaternion)
            translation = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]

            quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]

            # Convert to homogeneous transformation matrix (4x4)
            # print("Translation matrix:")
            # print(translation_matrix(translation))

            # print("Rotation matrix:")
            # print(quaternion_matrix(quaternion))

            homogeneous_matrix = np.dot(translation_matrix(translation), quaternion_matrix(quaternion))
            # print("Transformation matrix:")
            # print(homogeneous_matrix)

            return homogeneous_matrix

        except Exception as e:
            self.etasl_node.get_logger().warn(f"Failed to lookup transform: {e}")

    def transform_to_matrix(transform_stamped):
        translation = transform_stamped.transform.translation
        quat = transform_stamped.transform.rotation

        # Convert quaternion to 4x4 rotation matrix
        rot_matrix = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])

        # Set the translation part
        rot_matrix[0:3, 3] = [translation.x, translation.y, translation.z]

        return rot_matrix


class RunOptimization(Generator):
    def __init__(self):
        super().__init__("RunOptimization",[SUCCEED])    
        self.etasl_node = BeTFSMNode.get_instance()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.etasl_node)

    def co_execute(self,blackboard):

        #TODO: put a good estimate here, specially the orientation part
        T_c_b_estimate = np.eye(4) #Estimate of pose robot base wrt camera
        T_ee_o_estimate = np.eye(4) #Estimate of pose tracked object wrt end effector


        T_b_ee = []
        T_c_o = []

        assert len(blackboard['calibration_poses_robot']) == blackboard['calibration_poses_camera'], "The number of elements in calibration_poses_robot and calibration_poses_camera should match"


        for pose in blackboard['calibration_poses_robot']:
            T_b_ee.append(pose)

        for pose in blackboard['calibration_poses_camera']:
            T_c_o.append(pose)

     
        T_c_b, T_ee_o = self.optimize(T_b_ee,T_c_o,T_ee_o_estimate,T_c_b_estimate)

        print("The solution for the estimated pose of base robot wrt camera is:")
        print(T_c_b)

        print("The solution for the estimated pose of the tracked object wrt end effector:")
        print(T_ee_o)


        # pdb.set_trace()
        
        yield SUCCEED
    def optimize(T_b_ee, T_c_o, T_ee_o_estimate = np.eye(4), T_c_b_estimate = np.eye(4)):
        """
        Specification and solution of multi-pose calibration problem in Casadi.
        
        Be aware that this is a nonlinear problem and therefore multiple solutions are possible.
        Using the 3rd and 4th argument you can give an initialization so that you converge to the correct solution.
        
        TODO: rewrite this with optistack to make it clearer
        
        Parameters:
            T_b_ee ([numpy.array()]): list of N 4x4 pose matrices consisting of pose end effector wrt base robot
            T_c_o ([numpy.array()]): list of N 4x4 pose matrices consisting of measured pose tracked object wrt camera frame
            T_ee_o_estimate (numpy.array()): 4x4 pose matrix consisting of initial estimate for pose tracked object wrt end effector
            T_c_b_estimate (numpy.array()): 4x4 pose matrix consisting of initial estimate of base robot wrt camera frame
        
        Returns: 
            KDL.Frame: Fr_c_b_calib, estimated pose of base robot wrt camera
            KDL.Frame: Fr_ee_o_calib, estimated pose of object wrt end effector
        
        """
        
        # Define symbolic variables for the unknown poses (2 x 12 variables)
        T_c_b = cas.SX.sym('T_c_b',3,4)
        T_ee_o = cas.SX.sym('T_ee_o',3,4)
        X = cas.vertcat(cas.vec(T_c_b),cas.vec(T_ee_o)) # vectorize and stack variables
        
        # Homogeneous matrix extension necessary for pose multiplication
        T_c_b_homog = cas.vertcat(T_c_b,np.matrix([0,0,0,1]))
        T_ee_o_homog = cas.vertcat(T_ee_o,np.matrix([0,0,0,1]))
        
            # Debugging
        #    print T_c_b_homog
        #    print T_b_ee[1]
        #    print T_ee_o_homog
        #    print T_c_o[1]
        #    print T_ee_o_estimate
            
            # Initial values of unknown variables
        #    T_ee_o_init =  np.matrix([[1,0,0,-0.05],[0,0,1,0.0],[0,-1,0,0.08]]) 
        #    T_ee_o_init = np.matrix([[1,0,0,1.2],[0,1,0,0],[0,0,1,0]]) # using numpy
        #    T_b_c_init =  np.matrix([[1,0,0,0.3],[0,1,0,2.0],[0,0,1,1.0]]) # using numpy
        #    T_ee_o_init =  np.matrix([[0, -1, 0, -0.05],[-1, 0, 0, 0.0],[0, 0, -1, 0.9]])
        #    T_b_c_init =  np.matrix([[-1,0,0,0.65],[0,0,-1,1.53],[0,-1,0,1.35]]) # first calib lighthouse estimate
        #    T_b_c_init =  np.matrix([[-1,0,0,0.65],[0,0,-1,1.53],[0,-1,0,0.90]]) # second calib lighthouse estimate
        #    T_b_c_init =  np.matrix([[-1,0,0,1.20],[0,0,-1,-1.25],[0,-1,0,0.90]]) # second calib lighthouse estimate
        #    T_b_c_init =  np.matrix([[-1,0,0,1.20],[0,0,-1,-1.25],[0,-1,0,1.40]]) # second calib lighthouse estimate
        #    T_b_c_init = T_b_c_init[0:3][0:3]
        
        # Define objective function
        f = 0 
            
        # Optimize for every calibration position
        for i in range(len(T_c_o)):
            
            
            # define error on the pose loop {b} -> {ee} -> {o} -> {c} -> {b}
            # DeltaT = T_c_b * T_b_ee * T_ee_o * T_c_o
            DeltaT = cas.mtimes(T_c_b_homog, cas.mtimes(T_b_ee[i], cas.mtimes(T_ee_o_homog, cas.inv(T_c_o[i])))) - cas.SX_eye(4)
            
            # add squared error to objective function
            # (you can still also add separate weights on rotation/translation)
            for ii in range(4):
                for jj in range(4):
                    f += DeltaT[ii,jj]**2
            
        # add orthonormality constraints on pose variables: RR^T = I_3 
        g = [(cas.vec(cas.mtimes(T_c_b[0:3,0:3],cas.transpose(T_c_b[0:3,0:3])) - cas.SX_eye(3) ))] # R*R^T - I_3 = 0_3
        g.append(cas.vec(cas.mtimes(T_ee_o[0:3,0:3],cas.transpose(T_ee_o[0:3,0:3])) - cas.SX_eye(3))) # R*R^T - I_3 = 0_3
        lbg = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # lower bound constraint, don't forget!
        ubg = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # upper bound constraint, don't forget!
        
        # setting up the nlp symbolically
        nlp = {'x':X,'f':f,'g':cas.vertcat(*g)}
        opts = {'ipopt': {'max_iter':1000, 'tol':1e-15, 'print_level':5}};
        S = cas.nlpsol('S','ipopt',nlp,opts)
        
        # solve the NLP by providing the initial point
        r = S(x0 = cas.vertcat(cas.vec(cas.vertcat(T_c_b_estimate[0:3][0:3])), cas.vec(cas.vertcat(T_ee_o_estimate[0:3][0:3]))), lbg=lbg, ubg=ubg)
        #r = S(x0 = vertcat(vec(vertcat(T_c_b_estimate[0:3][0:4])), vec(vertcat(T_ee_o_estimate[0][0:3][0:4]))), lbg=lbg, ubg=ubg)
            
        x_opt = r['x']
        f_opt = float(r['f'])        
        print("pose loop error: " + f_opt)
        
        T_c_b_calib = cas.reshape(x_opt[0:12],3,4).full()
        T_ee_o_calib = cas.reshape(x_opt[12:],3,4).full()
        
        # Build frames from solution of nlp      
        # temp_frame = KDL.Frame()    
        # temp_frame.M = KDL.Rotation(float(T_c_b_calib[0,0]),float(T_c_b_calib[0,1]),float(T_c_b_calib[0,2]),
        #                             float(T_c_b_calib[1,0]),float(T_c_b_calib[1,1]),float(T_c_b_calib[1,2]),
        #                             float(T_c_b_calib[2,0]),float(T_c_b_calib[2,1]),float(T_c_b_calib[2,2]))
        # temp_frame.p = KDL.Vector(float(T_c_b_calib[0,3]),float(T_c_b_calib[1,3]),float(T_c_b_calib[2,3]))
        # T_c_b_calib = temp_frame
        # print 'T_c_b_calib:'
        # print T_c_b_calib
        
        # temp_frame = KDL.Frame()  
        # temp_frame.M = KDL.Rotation(float(T_ee_o_calib[0,0]),float(T_ee_o_calib[0,1]),float(T_ee_o_calib[0,2]),
        #                             float(T_ee_o_calib[1,0]),float(T_ee_o_calib[1,1]),float(T_ee_o_calib[1,2]),
        #                             float(T_ee_o_calib[2,0]),float(T_ee_o_calib[2,1]),float(T_ee_o_calib[2,2]))
        # temp_frame.p = KDL.Vector(float(T_ee_o_calib[0,3]),float(T_ee_o_calib[1,3]),float(T_ee_o_calib[2,3]))
        # T_ee_o_calib = temp_frame
        # print 'T_ee_o_calib:'
        # print T_ee_o_calib

        return T_c_b_calib, T_ee_o_calib



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



# main
def main(args=None):

    print("betfsm")
    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("camera_calibration")

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    #set_logger("state",my_node.get_logger())

    blackboard = {}

    load_task_list("$[etasl_ros2_application_template]/skill_specifications/libraries/robelix/tasks/camera_calibration.json",blackboard)
    
    sm  = Sequence("CalibrationRoutine", children=[
                                                    eTaSL_StateMachine("MovingHome","MovingHome",node=None),
                                                    eTaSL_StateMachine("calibration_pose_1","calibration_pose_1",node=None),
                                                    StorePose(),
                                                    eTaSL_StateMachine("calibration_pose_2","calibration_pose_2",node=None),
                                                    StorePose(),
                                                    eTaSL_StateMachine("calibration_pose_3","calibration_pose_3",node=None),
                                                    StorePose(),
                                                    eTaSL_StateMachine("calibration_pose_4","calibration_pose_4",node=None),
                                                    StorePose(),
                                                    eTaSL_StateMachine("calibration_pose_5","calibration_pose_5",node=None),
                                                    StorePose(),
                                                    eTaSL_StateMachine("calibration_pose_6","calibration_pose_6",node=None),
                                                    StorePose(),
                                                    RunOptimization()
    ])

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
