#!/usr/bin/env python3
#
# Opens a door about 70 degrees with the gripper of a Franka Emika Panda (setup franka_panda_door_opening.setup.json, environment
# door_opening/door_with_handle.xml), inspired on door_and_drawer.py. The door is free standing, 0.5 m wide and 1.5 m high (no frame),
# its vertical hinge is at (0.20, 0.10) in the base frame of the robot (the robot stands 0.1 m further along +y than the hinge: its
# base is at y = 0.1 in the world); the closed door is in the plane x = 0.20 and extends from the hinge to the right of the
# robot; it rotates about +z, away from the robot (towards +x). The handle, a cube of 5 cm on a short stem, is 0.40 m from the hinge,
# 0.333 m above the base of the robot: its center is at (0.156, -0.30, 0.333) in the base frame when the door is closed.
#
# The gripper is perpendicular to the handle: its z axis is horizontal and along the normal of the door (+x) and its fingers close
# along the y axis, in the direction of the door, so the fingers clamp the sides of the cube. (With the fingers above each other the
# arm would need joints at their limits: checked with inverse kinematics; with the fingers closing sideways the whole arc of the
# handle, 0 to 70 degrees, is reached with a margin of 12 % of the range of the joints. The home, pregrasp and grasp poses are free
# of collisions, also of the robot with itself.)
#
#   approach:  home, a joint space move to a pose 10 cm in front of the handle (DoorPregrasp: centered on the handle: the
#              position of the handle is known), then force control (ApproachHandle = move_compliant_along_axis): the gripper
#              moves slowly along its own z axis (the normal of the door), compliant sideways,
#              until the hand touches the cube (3 N); the contact is the reference for the depth: BackOff moves back 1.2 cm;
#   grasp:     the gripper closes on the handle (CANCEL if nothing is between the fingers: the skill ends with CANCEL);
#   open:      force control (PushDoor = move_compliant_along_axis) in the task frame of the task frame formalism: its
#              origin is the tool center point of the gripper (fer_hand_tcp, in the middle of the handle) and its z axis the
#              tangent of the arc of the handle (tf_2_insframe: the hand turned 3 degrees about its y axis). The gripper
#              pushes along this axis with 20 N, compliant along its x and y axes (desired force 0) and with the torques controlled
#              to 0 around all axes: the handle forces the tool to move on a circular arc and the tool turns with the door.
#              The task finishes when the path length along the z axis of the task frame, computed by an integrator (a feature
#              variable and a constraint), reaches "distance" of PushDoor in the task list: 0.445 m is 70 degrees (the radius
#              of the handle is 0.40 m, the tool center point about 0.4 m from the hinge);
#   release:   the gripper opens, moves back 12 cm along its z axis and the robot returns home.
#
# Only for the simulation: the BeTFSM states of fake_gripper_control.py command the simulated gripper of the Crospi Playground
# ("gripper" of the setup) over ROS 2 topics of the simulator; for a real gripper, states for its driver replace them.

import os
import sys

import rclpy
from betfsm import Message, Sequence, get_logger, set_logger
from betfsm_crospi import CrospiTask, load_task_list
from betfsm_ros import BeTFSMNode, ROSRunner, ServiceClient
from std_srvs.srv import Trigger

from fake_gripper_control import GripperClose, GripperOpen

GRASP_FORCE = 40.0             # per finger [N]


class DoorOpening(Sequence):
    def __init__(self):
        super().__init__("door_opening", [
            Message(None, msg="door_opening: open the door"),
            GripperOpen("open gripper"),
            CrospiTask("MovingHome", "MovingHome"),
            CrospiTask("in front of the door handle", "DoorPregrasp"),
            ServiceClient("tare F/T sensor", "/mujoco_robotsimulator/tare_ft_sensor", Trigger, []),  # force control: the sensor starts at zero
            CrospiTask("approach the handle (until contact)", "ApproachHandle"),
            CrospiTask("back off from the contact", "BackOff"),
            GripperClose("close gripper", force=GRASP_FORCE),   # SUCCEED: the handle is between the fingers
            CrospiTask("push the door open (force control, torques 0)", "PushDoor"),
            GripperOpen("release the handle"),
            CrospiTask("retreat", "RetreatFromHandle"),
            CrospiTask("MovingHome at the end", "MovingHome"),
        ])


def main(args=None):
    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("door_opening")
    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())
    get_logger().info("door_opening started")
    blackboard = {}

    # Task list: by default the .json file with the name of this script; another one (e.g. the task list of another
    # robot) can be given as the first argument: python3 <this script> <task list: a file name or a complete path>
    # It is taken out of sys.argv: the runner of BeTFSM parses the other arguments (its options, e.g. --serve).
    json_name = sys.argv.pop(1) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else os.path.splitext(os.path.basename(__file__))[0] + ".json"
    load_task_list(json_name, blackboard)
    
    runner = ROSRunner(my_node, DoorOpening(), blackboard, frequency=100.0, publish_frequency=5.0, debug=False,
                       display_active=False)
    try:
        runner.run()
    except KeyboardInterrupt:
        my_node.destroy_node()
        return
    my_node.destroy_node()
    rclpy.shutdown()
    print("shutdown")


if __name__ == "__main__":
    sys.exit(main(sys.argv))
