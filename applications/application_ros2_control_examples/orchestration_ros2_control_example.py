#  Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Santiago Iregui
#  email: <santiago.iregui@kuleuven.be>
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.

"""
Runs the demo motion of the ros2_control examples once: MoveToTarget, SineWaveLastJoint, MoveBack.

Usage (with a crospi node running one of the *_ros2_control*.setup.json files of this folder):
    python3 orchestration_ros2_control_example.py ur10e         # task list ur10e_ros2_control_tasks.json
    python3 orchestration_ros2_control_example.py widowx_ai     # task list widowx_ai_ros2_control_tasks.json
"""

import sys

import rclpy

from betfsm import Sequence, Message, get_logger, set_logger
from betfsm_crospi import load_task_list, CrospiTask
from betfsm_ros import BeTFSMNode, ROSRunner


class DemoMotion(Sequence):
    def __init__(self):
        super().__init__("ros2_control_demo_motion", [
            Message(None, msg="Moving to the target configuration"),
            CrospiTask("MoveToTarget", "MoveToTarget"),
            Message(None, msg="Sine wave on the last joint"),
            CrospiTask("SineWaveLastJoint", "SineWaveLastJoint"),
            Message(None, msg="Moving back"),
            CrospiTask("MoveBack", "MoveBack"),
            Message(None, msg="Demo motion finished"),
        ])


def main(args=None):
    # Removed from sys.argv, since the ROSRunner parses the command line arguments
    robot = sys.argv.pop(1) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else "ur10e"
    if robot not in ("ur10e", "widowx_ai"):
        print("The robot must be ur10e or widowx_ai")
        return 1

    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("ros2_control_demo_motion")
    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())

    blackboard = {}
    load_task_list(f"$[crospi_application_template]/applications/application_ros2_control_examples/{robot}_ros2_control_tasks.json", blackboard)

    runner = ROSRunner(my_node, DemoMotion(), blackboard, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
    try:
        runner.run()
    except KeyboardInterrupt:
        pass
    my_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
