import os
import rclpy
import sys



from betfsm import (
    Sequence, Repeat,  Message, SUCCEED, TICKING, CANCEL, ABORT, get_logger,set_logger
)
from betfsm_crospi import load_task_list, CrospiTask
from betfsm_ros import BeTFSMNode,ROSRunner

# Add the directory containing the skill module to sys.path
_SKILL_DIR = os.path.join(
    os.path.dirname(__file__),
    '../../skill_specifications/libraries/skill_lib_example/skill_for_debug_sine_wave_last_joint'
)
sys.path.append(os.path.abspath(_SKILL_DIR))

from skill_for_debug_sine_wave_last_joint import MySequence as SkillForDebugSineWaveLastJoint

class MyTree(Repeat):
    def __init__(self):
        sequence = Sequence("my_sequence", [
            Message(None,msg="start of a new loop"),
            SkillForDebugSineWaveLastJoint()
        ])
        super().__init__("my_tree",-1,sequence) # -1 for infinite loop, 1 for one time through the sequence, 2 for two times, etc.


def main(args=None):
    rclpy.init(args=args)    
    my_node = BeTFSMNode.get_instance("skill_example")
    set_logger("default",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())
    get_logger().info("application example (composition of skills) started")
    blackboard = {}

    #Use the following to load the task list relative to a ROS2 package, e.g. crospi_application_template
    load_task_list("$[crospi_application_template]/skill_specifications/libraries/skill_lib_example/skill_for_debug_sine_wave_last_joint/skill_for_debug_sine_wave_last_joint.json",blackboard)


    sm = MyTree()
    runner = ROSRunner(my_node,sm,blackboard, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
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
