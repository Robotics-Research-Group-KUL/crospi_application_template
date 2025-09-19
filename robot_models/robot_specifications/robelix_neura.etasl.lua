require("context")
require("geometric")
local ament = require("libamentlua")
-- worldmodel=require("worldmodel")
local urdfreader=require("urdfreader")

local M = {}

--
-- read robot model:
--

local etasl_application_share_dir = ament.get_package_share_directory("etasl_ros2_application_template")
local xmlstr = urdfreader.loadFile(etasl_application_share_dir .. "/robot_models/urdf_models/robot_setups/robelix/use_case_setup_robelix_mobile.urdf")
local robot_worldmodel = urdfreader.readUrdf(xmlstr,{})
-- robot:writeDot("ur10_robot.dot")
local VL = {}
local frames = robot_worldmodel:getExpressions(VL,ctx,{
                                                            tcp_frame={'maira7M_flange','world'}, --Frame at the end effector, attached to the robot, wrt world
                                                            tcp_frame_wrt_link2={'maira7M_flange','maira7M_link2'}, --Frame at the end effector, attached to the robot, wrt the link2
                                                            tcp_frame_wrt_mav={'maira7M_flange','mav_base_link'}, -- Frame at the end effector, attached to the robot, wrt center of mobile mav
                                                            tool0_cables_mav={'tool0_cables','mav_base_link'}, -- Frame at the end effector, attached to the robot, wrt center of mobile mav
                                                            tool0_cables_root={'tool0_cables','maira7M_root_link'}, -- Frame at the end effector, attached to the robot, wrt center of mobile mav
                                                            tool0_cables_world={'tool0_cables','world'}, -- Frame at the end effector, attached to the robot, wrt center of mobile mav
                                                            tool0_insertions_root={'tool0_insertions','maira7M_root_link'}, -- Frame at the end effector, attached to the robot, wrt center of mobile mav
                                                            linear_axis_base={'linear_axis_base','world'}, --Frame at the robot base, attached to mobile mav, wrt world
                                                            mav_base_link={'mav_base_link','world'}, --Frame at the center of mobile mav, attached to mobile mav, wrt world
                                                            FT_sensor_frame = {'FT_sensor_frame','maira7M_root_link'}
                                                        })

-- The following is the kinematic constraint of a differential drive robot (which cannot move in the lateral direction)
task_frame_inst_mobile_base = inv(make_constant(frames["mav_base_link"]))*frames["mav_base_link"]

Constraint{
    context = ctx,
    name    = "mav_lateral_kinematic_constraint",
    expr    = coord_y(origin(task_frame_inst_mobile_base)),
    target  = constant(0)*time, -- Zero velocity constraint
    K       = 0,
    weight  = 1,
    priority= 0
};


-- roll_mobile,pitch_mobile, yaw_mobile = getRPY( rotation(task_frame_inst_mobile_base)) --TODO: replace yaw_mobile directly by joint variable theta


theta_mobile_joint = ctx:getScalarExpr("theta_mobile_joint")
ctx:setOutputExpression("omega_mav", theta_mobile_joint)
ctx:setOutputExpression("velocity_mav", coord_x(origin(task_frame_inst_mobile_base)))


M.frames= frames
M.xmlstr = xmlstr
M.robot_worldmodel = robot_worldmodel
M.urdfreader = urdfreader

return M