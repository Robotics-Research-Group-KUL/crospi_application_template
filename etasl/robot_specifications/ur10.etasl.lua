require("context")
require("geometric")
-- worldmodel=require("worldmodel")
urdfreader=require("urdfreader")

--
-- read robot model:
--

handle = io.popen("ros2 pkg prefix etasl_ros2_application_template")
etasl_application_prefix_dir = handle:read("*a")
handle:close()
etasl_application_prefix_dir = etasl_application_prefix_dir:gsub("%s+$", "") -- Trim any trailing newline

etasl_application_share_dir = etasl_application_prefix_dir .. "/share/etasl_ros2_application_template"

xmlstr = urdfreader.loadFile(etasl_application_share_dir .. "/robot_description/urdf/ur10/use_case_setup_ur10.urdf")
robot = urdfreader.readUrdf(xmlstr,{})
-- robot:writeDot("ur10_robot.dot")
VL = {}
rv = robot:getExpressions(VL,ctx,{ee={'tool0','base_link'}})
ee = rv['ee']
robot_joints={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"}