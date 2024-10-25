require("context")
require("geometric")
ament = require("libamentlua")

-- worldmodel=require("worldmodel")
urdfreader=require("urdfreader")

--
-- read robot model:
--

etasl_application_share_dir = ament.get_package_share_directory("etasl_ros2_application_template")
xmlstr = urdfreader.loadFile(etasl_application_share_dir .. "/robot_description/urdf/kuka_iiwa/use_case_setup_iiwa.urdf")
robot = urdfreader.readUrdf(xmlstr,{})
-- robot:writeDot("kuka_iiwa_robot.dot")
VL = {}
rv = robot:getExpressions(VL,ctx,{ee={'right_tool0','world'}})
ee = rv['ee']
robot_joints={"right_joint_a1","right_joint_a2","right_joint_a3","right_joint_a4","right_joint_a5","right_joint_a6","right_joint_a7"}
