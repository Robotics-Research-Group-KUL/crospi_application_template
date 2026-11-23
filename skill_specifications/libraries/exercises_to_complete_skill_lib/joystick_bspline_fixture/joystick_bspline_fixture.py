#!/usr/bin/env python3
#
# Joystick teleoperation along a B-spline virtual fixture (Crospi playground example):
#   Sequence: move home, move 5 cm up, then teleoperate the tool with the 6D joystick along a horn-shaped virtual fixture
#   (move_joystick_bspline_fixture of core_task_lib): the tool stays on a quadratic B-spline from the base of the
#   fixture to its top disk, and can be kept tangent to it.
#
# In the playground the joystick is the emulated SpaceMouse of the 3D view (input channel "joystick_input"):
# right-drag on its panel (or Ctrl + right-drag in the 3D view) moves the tool, left-drag rotates it.
# Task lists: joystick_bspline_fixture.json (UR10e, default) and joystick_bspline_fixture_widowx_ai.json (WidowX AI,
# given as the argument). The marker "fixture" shows the control points of the spline (the expressions "fixture" of the task specification).
# move_joystick_bspline_fixture runs until the skill is stopped (execution_time = 0 in the task list).

import os
import sys

import rclpy
from betfsm import Sequence, Message, get_logger, set_logger
from betfsm_crospi import load_task_list, CrospiTask
from betfsm_ros import BeTFSMNode, ROSRunner
from crospi_playground.betfsm_markers import MarkerEdit
# Breakpoint of the Crospi Playground: uncomment to pause the playground in a state, e.g.
#   PlaygroundBreakpoint("check pose", message="Check the pose of the tool, then Resume")
# (does nothing outside the playground)
# from crospi_playground.betfsm_debug import PlaygroundBreakpoint


class JoystickBsplineFixture(Sequence):
    def __init__(self):
        super().__init__("joystick_bspline_fixture", [
            CrospiTask("MovingHome", "MovingHome"),
            CrospiTask("MovingRelative", "MoveRelativeCartesian"),
            Message(None, msg="Move the tool with the SpaceMouse along the fixture (right-drag: move, left-drag: rotate)"),
            MarkerEdit(json='{"name":"fixture","position":[0,0,0],"is-path":true,"points":{"etasl":"fixture"},"interpolation":"bspline","degree":2,"reference":"base_link"}'),
            CrospiTask("JoystickFixture", "joystick_bspline_fixture"),
        ])


def main(args=None):
    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("joystick_bspline_fixture")
    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())
    get_logger().info("Joystick teleoperation with a B-spline fixture started")
    blackboard = {}

    # Task list: by default the .json file with the name of this script; another one (e.g. the task list of another
    # robot) can be given as the first argument: python3 <this script> <task list: a file name or a complete path>
    # It is taken out of sys.argv: the runner of BeTFSM parses the other arguments (its options, e.g. --serve).
    json_name = sys.argv.pop(1) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else os.path.splitext(os.path.basename(__file__))[0] + ".json"
    load_task_list(json_name, blackboard)

    sm = JoystickBsplineFixture()
    runner = ROSRunner(my_node, sm, blackboard, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
    try:
        runner.run()
    except KeyboardInterrupt:
        pass
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
