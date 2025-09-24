require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")

reqs = require("task_requirements")
-- TODO
task_description = "Move towards a cartesian frame while sliding through the cable maintaining a desired tension"

-- ========================================= FUNCTIONS ===================================
function dead_zone(signal,dead_val)
    signal_dead_zone = conditional(abs(signal)-dead_val, signal + conditional(signal, -dead_val, dead_val), constant(0))
    return signal_dead_zone
end
-- print("Que se iceee============================ 0000000")
-- ========================================= PARAMETERS ===================================
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration [m/s2]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxvel_mav", description="Maximum linear velocity for MAV m/s", default = 0.1, required=true, maximum = 0.3}),
    reqs.params.scalar({name="maxacc_mav", description="Maximum linear acceleration for MAV m/s^2", default = 0.1, required=true, maximum = 0.3}),
    reqs.params.scalar({name="cable_tens", description="Desired Cable Tension [N]", default = 5.0, required=true, maximum = 15}),
    reqs.params.scalar({name="force_threshold", description="Force dead zone [N]", default = 0.5, required=false}),
    reqs.params.scalar({name="torque_threshold", description="Torque dead zone [Nm]", default = 0.05, required=false}),
    reqs.params.array({name="tool_COG", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, 
                            description="Array with the center of gravity of the tool w.r.t FT_sensor_frame [m]", required=true, minItems = 3, maxItems = 3}),
    reqs.params.scalar({name="tool_weight", description="Weight of the tool attached to the end-effector [N]", default = 0.0, required=true}),
    reqs.params.string({name="task_frame_world", description="Task frame from world to tcp", default = "tcp_frame", required=false}),
    reqs.params.string({name="task_frame_mav", description="Task frame from mav to tcp", default = "tcp_frame_wrt_mav", required=false}),
    reqs.params.string({name="FT_sensor_frame", description="Name of frame where the forces and torques a measured", default = "FT_sensor_frame", required=true}),
    reqs.params.scalar({name="a", description="Cable interaction parameter a", default=2.5,  required=true, minimum = 1.0, maximum = 5.0}),
    reqs.params.scalar({name="C", description="Cable interaction parameter C", default=0.5,  required=true, minimum = 0.0, maximum = 1.0}),
    reqs.params.scalar({name="turning_dir", description="Turning direction of the cable sliding", default=1.0,  required=true, minimum = -1.0, maximum = 1.0}),
    reqs.params.array({name="desired_pos", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, description="Array with the desired position of the task frame in [x,y,z] w.r.t. board", required=true, minimum = -1.5, maximum = 1.5, minItems = 3, maxItems = 3}),
})

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame_world"), --The frame is selected as a parameter, to make the skill even more reusable
    param.get("task_frame_mav"), --The frame is selected as a parameter, to make the skill even more reusable
    param.get("FT_sensor_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    -- "forearm"
    -- "tcp_frame"
    "tool0_cables_debug_root",
    "mav_base_link",
    "tool0_cables_mav"
    --Add all frames that are required by the task specification
})


task_frame_world = robot.getFrame(param.get("task_frame_world"))
task_frame_mav  = robot.getFrame(param.get("task_frame_mav"))
FT_sensor_frame = robot.getFrame(param.get("FT_sensor_frame"))
mav_base_link  = robot.getFrame("mav_base_link")
tool0_cables_mav  = robot.getFrame("tool0_cables_mav")

debug_frame = robot.getFrame("tool0_cables_debug_root")

mav_pose_wrt_world   = ctx:createInputChannelFrame("mav_pose_wrt_world")


-- task_frame_world_without_mav_feedforward = make_constant(mav_base_link)*tool0_cables_mav
task_frame_world_without_mav_feedforward = mav_pose_wrt_world*tool0_cables_mav


-- ========================================= PARAMETERS ===================================
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))
maxvel_mav    = constant(param.get("maxvel_mav"))
maxacc_mav    = constant(param.get("maxacc_mav"))

cable_tens = constant(param.get("cable_tens"))
force_threshold = constant(param.get("force_threshold"))
torque_threshold = constant(param.get("torque_threshold"))
tool_weight = constant(param.get("tool_weight"))

a = constant(param.get("a"))
C = constant(param.get("C"))
turn_dir = constant(param.get("turning_dir"))

-- print("Que se iceee============================ 11111111")
tool_COG = param.get("tool_COG")
-- print("Que se iceee============================ 22222222")
tool_COG_x = constant(tool_COG[1])
tool_COG_y = constant(tool_COG[2])
tool_COG_z = constant(tool_COG[3])

-- print("Que se iceee============================ 33333333")
desired_pos = param.get("desired_pos")
-- print("Que se iceee============================ 44444444")
x_coordinate_board   = constant(desired_pos[1])
y_coordinate_board   = constant(desired_pos[2])
z_coordinate_board   = constant(desired_pos[3])

-- print("Que se iceee============================22222")
-- compute orientation from quaternion
target_position_board = vector(x_coordinate_board, y_coordinate_board, z_coordinate_board)
T_board_fixture = frame(target_position_board)

-- print("Que se iceee============================3333333")
-- ========================================= Variables coming from topic input handlers ===================================
sensed_wrench   = ctx:createInputChannelWrench("wrench_input")

T_world_board   = ctx:createInputChannelFrame("board_pose")
-- TODO: Should I use initial value? (To avoid jumps?)
-- T_world_fixture = T_world_board * T_board_fixture
T_world_fixture = initial_value(time, T_world_board) * T_board_fixture
target_position_tf_world = origin(T_world_fixture)
-- ===================================== TRANSFORM WRENCH TO TASK FRAME ================================
Fx_sensed = coord_x(force(sensed_wrench))
Fy_sensed = coord_y(force(sensed_wrench))
Fz_sensed = coord_z(force(sensed_wrench))
Tx_sensed = coord_x(torque(sensed_wrench))
Ty_sensed = coord_y(torque(sensed_wrench))
Tz_sensed = coord_z(torque(sensed_wrench))

Fx_dead_zone = dead_zone(Fx_sensed, force_threshold)
Fy_dead_zone = dead_zone(Fy_sensed, force_threshold)
Fz_dead_zone = dead_zone(Fz_sensed, force_threshold)
Tx_dead_zone = dead_zone(Tx_sensed, torque_threshold)
Ty_dead_zone = dead_zone(Ty_sensed, torque_threshold)
Tz_dead_zone = dead_zone(Tz_sensed, torque_threshold)

-- =============================== GRAVITY COMPENSATION ==============================
-- d_g = vector(0,0,-1) -- wrt to the base frame
-- FT_sensor_frame_to_cog = frame(vector(tool_COG_x, tool_COG_y, tool_COG_z))

-- -- This is the wrench removed by the taring and that needs to be compensated (Assuming it was tared aligned with the gravity vector)
-- virtual_wrench_wrt_base_frame = wrench(d_g*tool_weight, cross(origin(FT_sensor_frame_to_cog),d_g*tool_weight))
-- virtual_wrench_wrt_FT_frame = transform(rotation(FT_sensor_frame), virtual_wrench_wrt_base_frame)

-- -- This is the wrench caused by the tool weight that needs to be compensated in the FT_sensor_frame
-- -- Rotate the wrench to the FT_sensor_frame orientation
-- wrench_cog_ftframe = transform(rotation(inv(FT_sensor_frame)), wrench(d_g*tool_weight, vector(0,0,0)))

-- -- Translate the wrench to the FT_sensor_frame
-- wrench_FT_frame = ref_point(wrench_cog_ftframe, origin(inv(FT_sensor_frame_to_cog)))
-- wrench_dead_zone = wrench(vector(Fx_dead_zone,Fy_dead_zone,Fz_dead_zone),vector(Tx_dead_zone,Ty_dead_zone,Tz_dead_zone)) - wrench_FT_frame + virtual_wrench_wrt_FT_frame

-- WARNING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! This version works in the in Maira or IIWA but not in UR10e
d_g = vector(0,0,-1)
FT_sensor_frame_to_cog = frame(vector(tool_COG_x, tool_COG_y, tool_COG_z))

virtual_wrench_wrt_base_frame = wrench(d_g*tool_weight, cross(origin(FT_sensor_frame_to_cog),d_g*tool_weight))

wrench_cog_ftframe = transform(rotation(inv(FT_sensor_frame)), wrench(d_g*tool_weight, vector(0,0,0)))
wrench_FT_frame = ref_point(wrench_cog_ftframe, -origin(FT_sensor_frame_to_cog))

wrench_dead_zone = wrench(vector(Fx_dead_zone,Fy_dead_zone,Fz_dead_zone),vector(Tx_dead_zone,Ty_dead_zone,Tz_dead_zone)) - wrench_FT_frame - virtual_wrench_wrt_base_frame

-- =============================== TRANSLATE FT TO TASK_FRAME ==============================

wrench_task_frame   = ref_point(transform(rotation(inv(task_frame_world_without_mav_feedforward)*FT_sensor_frame), wrench_dead_zone) , -origin(inv(task_frame_world_without_mav_feedforward)*FT_sensor_frame))

Fx = coord_x(force(wrench_task_frame))
Fy = coord_y(force(wrench_task_frame))
Fz = coord_z(force(wrench_task_frame))
Tx = coord_x(torque(wrench_task_frame))
Ty = coord_y(torque(wrench_task_frame))
Tz = coord_z(torque(wrench_task_frame))

-- =============================== INSTANTANEOUS FRAME ==============================
task_frame_inst = inv(make_constant(task_frame_world_without_mav_feedforward))*task_frame_world_without_mav_feedforward
rot_vec = getRotVec(rotation(task_frame_inst))

-- =============================== TARGET POSE ====================================
start_pose = initial_value(time, task_frame_world_without_mav_feedforward)
start_pose_diff  = inv(start_pose)*task_frame_world_without_mav_feedforward
start_position  = origin(start_pose)
-- =============================== END POSE ==============================
end_position    = target_position_tf_world

-- ========================================== Imports utils_ts ============================
-- The following is done because utils_ts is a file of the library and not of the application ROS2 package.
local script_dir = debug.getinfo(1, "S").source:match("@(.*)/")
package.path = script_dir .. "/utilities/?.lua;" .. package.path  -- Add it to package.path
local utils_ts = require("utils_ts")

-- =========================== VELOCITY PROFILE ============================================
-- compute distances for displacements and rotations:
diff                    = cached(end_position-start_position)
diff, distance          = utils_ts.normalize( diff )

-- plan trapezoidal motion profile in function of time:
motion_profile = create_motionprofile_trapezoidal()
motion_profile:setProgress(time)
motion_profile:addOutput(constant(0), distance, maxvel, maxacc)

offset_mav = 0.1
target_x_coordinate_mav = x_coordinate_board + offset_mav
motion_profile:addOutput( initial_value(time, coord_x(origin(mav_base_link))), target_x_coordinate_mav, maxvel_mav, maxacc_mav)


maxvel_mav_theta = constant(0.1)
maxacc_mav_theta = constant(0.1)

initial_rot_mav = initial_value(time, rotation(mav_base_link))
target_R_mav = rotation(T_world_board)
diff_rot_mav                = cached(  getRotVec( inv(initial_rot_mav)*target_R_mav )) -- eq. axis of rotation for rotation from start to end:w
diff_rot_mav, angle_mav         = utils_ts.normalize( diff_rot_mav )
motion_profile:addOutput(constant(0), angle_mav, maxvel_mav_theta, maxacc_mav_theta)



d = get_output_profile(motion_profile, 0)            -- progression in distance
target_position_profile = start_position + diff*d

-- =============================== CONSTRAINT SPECIFICATION ==============================
Constraint{
	context=ctx,
	name="follow_path_x",
	expr = coord_x(origin(task_frame_world_without_mav_feedforward)),
	target = coord_x(target_position_profile),
	K = 2,
	priority = 2,
	weight = 50
};

Constraint{
	context=ctx,
	name="follow_path_y",
	expr = coord_y(origin(task_frame_world_without_mav_feedforward)),
	target = coord_y(target_position_profile),
	K = 2,
	priority = 2,
	weight = 50
};

Constraint{
	context=ctx,
	name="follow_path_z",
	expr = coord_z(origin(task_frame_world_without_mav_feedforward)),
	target = coord_z(target_position_profile),
	K = 2,
	priority = 2,
	weight = 50
};

-- moment_sign = conditional(abs(Tz)-0.0001,Tz/abs(Tz),-turn_dir)
-- turning_dir = conditional(time-2, moment_sign, -turn_dir)
-- turning_dir = -turn_dir

end_time_ramp = 1
K_max = 0.5 -- Real Robot 0.5 Simulation:0.0
K_ramp = conditional(time-end_time_ramp, K_max, K_max*time/end_time_ramp)
-- Constraint{
-- 	context=ctx,
-- 	name="follow_cable_tens",
-- 	model = turn_dir*(log(C)+a*coord_z(rot_vec)),
-- 	meas = log(sqrt(Fx*Fx + Fy*Fy)+0.1),
-- 	target = log(cable_tens+0.1),
-- 	K = K_ramp,
-- 	priority = 2,
-- 	weight = 1,
-- };

-- last_joint   = ctx:getScalarExpr(robot.robot_joints[7])
-- TODO:Change names and inputs
debug_frame_inst = inv(make_constant(debug_frame))*debug_frame
rot_vec_debug = getRotVec(rotation(debug_frame_inst))

Constraint{
	context=ctx,
	name="follow_cable_tens",
	model = turn_dir*(log(C)+a*coord_z(rot_vec_debug)),
	meas = log(sqrt(Fx*Fx + Fy*Fy)+0.1),
	target = log(cable_tens+0.1),
	K = K_ramp,
	priority = 2,
	weight = 100,
};

-- Constraint{}

-- Constant Orientation Maira
feature_variable_z_rotation_maira = Variable{context=ctx, name="feature_variable_z_rotation_maira", initial_value=0.0}
rotation_control_maira = rotation(task_frame_world_without_mav_feedforward)*rot_z(feature_variable_z_rotation_maira)
initial_orientation = initial_value(time, rotation(task_frame_world_without_mav_feedforward))
Constraint{
    context = ctx,
    name    = "constant_orientation_end_effector_wrt_world",
    expr    = inv(initial_orientation)*rotation_control_maira,
    K       = 3,
    weight  = 10,
    priority= 2
};

-- Constraint{
--     context = ctx,
--     name    = "x_angular",
--     expr    = coord_x(rot_vec) - 0*time,
--     K       = 0,
--     weight  = 100,
--     priority= 2
-- };

-- Constraint{
--     context = ctx,
--     name    = "y_angular",
--     expr    = coord_y(rot_vec) - 0*time,
--     K       = 0,
--     weight  = 100,
--     priority= 2
-- };

-- value_const = constant(0.0)
-- Constraint{
--     context  = ctx,
--     name     = "arm_workspace",
--     expr      = coord_x(origin(task_frame_mav)),
--     target_upper    = value_const,
--     K        = 1,
--     weight   = 2,
--     priority = 2
-- }

-- =========================== MAV CONSTRAINTS ============================================

function linear_weight(w_init,w_final,ind_var,ind_var_init, ind_var_final)
    local lin_weight = w_init - ((w_init-w_final)/(ind_var_final-ind_var_init))*(ind_var-ind_var_init)
    local weight = conditional(ind_var-ind_var_init,conditional(ind_var-ind_var_final,w_final,lin_weight),w_init)
    return weight
end



-- --------------------Constraints for MAV --------------------


print("Hola2")
threshold_mav_position = 0.03
-- variable_weight_mav_vel = linear_weight(constant(1) , constant(0), abs(coord_x(origin(mav_base_link)) - target_x_coordinate_mav),threshold_mav_position , threshold_mav_position + 0.01)
variable_weight_mav_vel = constant(0.0)
print("Hola3")
target_x_mav        = get_output_profile(motion_profile, 1) --get_output_profile is zero-index-based
Constraint{
    context     =ctx,
    name        ="mav_x_trapezoidal_profile",
    expr        = coord_x(origin(mav_base_link)) - target_x_mav,
    priority    = 2,
    weight      = 10000*(constant(1)-variable_weight_mav_vel),
    K           = 4
}
print("Hola4")
task_frame_inst_mobile_base = inv(make_constant(mav_base_link))*mav_base_link

Constraint{
    context = ctx,
    name    = "mav_x_zero_velocity_constraint",
    expr    = coord_x(origin(task_frame_inst_mobile_base)),
    target  = constant(0.0)*time, -- Zero velocity constraint
    K       = 0,
    weight  = 100000*variable_weight_mav_vel,
    priority= 2
};
print("Hola5")

theta_mav = ctx:getScalarExpr("theta_mobile_joint")

Constraint{
    context  = ctx,
    name     = "mav_theta_zero_velocity_constraint",
    expr     = theta_mav,
    target   = constant(0.0)*time, -- Zero velocity constraint
    K        = 0,
    weight   = 100000*variable_weight_mav_vel,
    priority = 2
};

-- --------------------Constraints for MAV orientation --------------------

feature_variable_pitch = Variable{context=ctx, name="feature_variable_pitch", initial_value=0.0}
feature_variable_roll  = Variable{context=ctx, name="feature_variable_roll", initial_value=0.0}
rotation_control_mav = rotation(mav_base_link)*rot_y(feature_variable_roll)*rot_x(feature_variable_pitch)

-- Constraint{
--     context  = ctx,
--     name     = "maintain_mav_alignment_with_board",
--     expr     = inv(rotation(T_world_board))*rotation_control_mav,
--     K        = 3,
--     weight   = 10,
--     priority = 2
-- };
-- Uncomment this to leave MAV orientation aligned with board following a trapezoidal motion provile (vision needed!!)

rot_mav_mp        = get_output_profile(motion_profile, 2) --get_output_profile is zero-index-based
targetrot_mav = initial_rot_mav*rotVec(diff_rot_mav,rot_mav_mp)

Constraint{
    context  = ctx,
    name     = "maintain_mav_alignment_with_board_trapezoidal",
    expr     = targetrot_mav*rotation_control_mav,
    K        = 3,
    weight   = 10000,
    priority = 2
};


-- variable_weight = linear_weight(constant(0) , constant(1), coord_x(origin(task_frame_mav)),value_const-0.1, value_const)


-- Constraint{
--     context = ctx,
--     name    = "mav_x_velocity_constraint",
--     expr    = coord_x(origin(task_frame_inst_mobile_base)),
--     target  = constant(0.05)*time, -- Zero velocity constraint
--     K       = 0,
--     weight  = 3*variable_weight,
--     priority= 2
-- };

-- =========================== MONITORS ============================================
local error_pos = norm(end_position-origin(task_frame_world_without_mav_feedforward))
local error_th = 1 --mm
Monitor{
    context=ctx,
    name='finish_after_motion',
    lower = error_th/1000,
    actionname = "exit",
    expr=error_pos
}

-- Monitor{
--     context=ctx,
--     name='debug',
--     upper = 0.5,
--     actionname = "debug",
--     expr=time
-- }

-- monitor_F = Monitor{context=ctx, 
-- 					name='finish_force',
-- 					upper=0.95,
-- 					actionname='exit', 
-- 					expr=abs(Fz)/contact_force
-- 				};

roll_tf, pitch_tf, yaw_tf = getRPY(rotation(task_frame_world_without_mav_feedforward))

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame_world_without_mav_feedforward)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame_world_without_mav_feedforward)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame_world_without_mav_feedforward)))

ctx:setOutputExpression("Fx"      ,Fx)
ctx:setOutputExpression("Fy"      ,Fy)
ctx:setOutputExpression("Fz"      ,Fz)
ctx:setOutputExpression("Tx"      ,Tx)
ctx:setOutputExpression("Ty"      ,Ty)
ctx:setOutputExpression("Tz"      ,Tz)
