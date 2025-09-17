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
                                                            linear_axis_base={'linear_axis_base','world'}, --Frame at the robot base, attached to mobile mav, wrt world
                                                            mav_base_link={'mav_base_link','world'} --Frame at the center of mobile mav, attached to mobile mav, wrt world
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

-- Defines the weight as a piecewise function of the ind_var.
-- If s<ind_var_init then weight = w_init
-- If  ind_var_init<s<ind_var_final then weight is a linear function
-- If s>ind_var_final then weight = w_final
function linear_weight(w_init,w_final,ind_var,ind_var_init, ind_var_final)
    local lin_weight = w_init - ((w_init-w_final)/(ind_var_final-ind_var_init))*(ind_var-ind_var_init)
    local weight = conditional(ind_var-ind_var_init,conditional(ind_var-ind_var_final,w_final,lin_weight),w_init)
    return weight
end
  

radius_fixture = 0.3 --meters
-- radius_fixture = 1.2 --meters
-- vector_fixture = vector(coord_x(origin(frames["tcp_frame_wrt_link2"])), coord_y(origin(frames["tcp_frame_wrt_link2"])) , coord_z(origin(frames["tcp_frame_wrt_link2"])))
vector_fixture_pre = vector(coord_x(origin(frames["tcp_frame_wrt_mav"])), coord_y(origin(frames["tcp_frame_wrt_mav"])) , coord_z(origin(frames["tcp_frame_wrt_mav"])))

vector_fixture = vector_fixture_pre - initial_value(time, vector_fixture_pre)
--   linear_weight(constant(1),constant(0),make_constant(norm(origin(tf) - virtual_fixture_path)),constant(1)*tube_radius_var+0.05, constant(1.5)*tube_radius_var+0.05)
weight_fixture = linear_weight(constant(1) , constant(0), norm(vector_fixture),radius_fixture-0.02, radius_fixture-0.015)

-- step_fixture_velocity = conditional(dot(vector_fixture, ),constant(1),constant(0)) -- step_fixture to make the end-effector stop moving whenever the joystick input is smaller than 0.001



Constraint{
    context = ctx,
    name    = "bounding_fixture_end_effector_coord_x_mobile",
    expr    = coord_x(origin(task_frame_inst_mobile_base)),
    target  = constant(0)*time, -- Zero velocity constraint
    K       = 0,
    weight  = weight_fixture*100000, --Deactivates motion of the mobile base when arm is within the fixture
    priority= 2
};

roll_mobile,pitch_mobile, yaw_mobile = getRPY( rotation(task_frame_inst_mobile_base))
Constraint{
    context = ctx,
    name    = "bounding_fixture_end_effector_theta_mobile",
    expr    = yaw_mobile,
    target  = constant(0)*time, -- Zero velocity constraint
    K       = 0,
    weight  = weight_fixture*100000, --Deactivates motion of the mobile base when arm is within the fixture
    priority= 2
};


Constraint{
    context = ctx,
    name    = "bounding_fixture_end_effector",
    expr    = norm(vector_fixture),
    target_upper  = radius_fixture, --Position constraint to bound the end effector within a spherical fixture 
    K       = 4,
    weight  = 10000000,
    priority= 2
};

ctx:setOutputExpression("omega_mav", yaw_mobile)
ctx:setOutputExpression("velocity_mav", coord_x(origin(task_frame_inst_mobile_base)))


M.frames= frames
M.xmlstr = xmlstr
M.robot_worldmodel = robot_worldmodel
M.urdfreader = urdfreader

return M