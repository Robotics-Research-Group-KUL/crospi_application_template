#!/usr/bin/env python3
#
# Joystick teleoperation (Crospi playground example):
#   Sequence: move home, then move the tool with the 6D joystick (move_joystick of core_task_lib).
#
# In the playground the joystick is the emulated SpaceMouse of the 3D view (input channel "joystick_input"):
# right-drag on its panel (or Ctrl + right-drag in the 3D view) moves the tool, left-drag rotates it.
# move_joystick runs until the skill is stopped (execution_time = 0 in the task list).

import os
import sys

import rclpy
from betfsm import Sequence, Message, get_logger, set_logger
from betfsm_crospi import load_task_list, CrospiTask
from betfsm_ros import BeTFSMNode, ROSRunner
from crospi_playground.betfsm_markers import MarkerEdit, MarkerUpdater, WithMarkers



class JoystickTeleoperation(Sequence):
    def __init__(self):
        super().__init__("joystick_teleoperation", [
            CrospiTask("MovingHome", "MovingHome"),
            Message(None, msg="Move the tool with the SpaceMouse (right-drag: move, left-drag: rotate)"),
            MarkerEdit(name="cone", json='{"is-cone":true,"name":"cone","position":{"etasl":"vector(origin_vf_x, origin_vf_y, origin_vf_z)"},"radius1":{"etasl":"rad1"},"radius2":{"etasl":"rad2"},"length":{"etasl":"length"}}'),
            CrospiTask("joystick_virtual_fixture", "joystick_virtual_fixture"),
        ])


def main(args=None):
    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("joystick_teleoperation")
    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())
    get_logger().info("Joystick teleoperation example started")
    blackboard = {}

    # the task list has the same base name as this file
    json_name = os.path.splitext(os.path.basename(__file__))[0] + ".json"
    load_task_list(json_name, blackboard)

    sm = JoystickTeleoperation()
    runner = ROSRunner(my_node, sm, blackboard, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
    try:
        runner.run()
    except KeyboardInterrupt:
        pass
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
