require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")

reqs = require("task_requirements")
-- TODO
task_description = "Move towards a pivot fixture while adapting the orientation based on the measured forces"

-- ========================================= FUNCTIONS ===================================
function dead_zone(signal,dead_val)
    signal_dead_zone = conditional(abs(signal)-dead_val, signal + conditional(signal, -dead_val, dead_val), constant(0))
    return signal_dead_zone
end

-- ========================================= PARAMETERS ===================================
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration [m/s2]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="eq_r", description="Equivalent radius for tap profile planning", default = 0.15, required=true, minimum = 0.1}),
    reqs.params.scalar({name="cable_tens", description="Desired Cable Tension [N]", default = 5.0, required=true, maximum = 15}),
    reqs.params.scalar({name="force_threshold", description="Force dead zone [N]", default = 0.5, required=false}),
    reqs.params.scalar({name="torque_threshold", description="Torque dead zone [Nm]", default = 0.05, required=false}),
    reqs.params.array({name="tool_COG", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, 
                            description="Array with the center of gravity of the tool w.r.t. FT_sensor_frame [m]", required=true, minItems = 3, maxItems = 3}),
    reqs.params.scalar({name="tool_weight", description="Weight of the tool attached to the end-effector [N]", default = 0.0, required=true}),
    reqs.params.scalar({name="k_x", description="Stiffness in the cable pulling direction", default=2700,  required=true, minimum = 1500, maximum = 4000}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=true}),
    reqs.params.string({name="FT_sensor_frame", description="Name of frame where the forces and torques a measured", default = "FT_sensor_frame", required=true}),
    reqs.params.scalar({name="z_down", description="Heigh gain", default=0.0,  required=true}),
    reqs.params.array({name="previous_to_current_fixture", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, description="Array with the position of the next fixture", required=true, minimum = -1.5, maximum = 1.5, minItems = 3, maxItems = 3}),
    reqs.params.array({name="T_root_board", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, description="Array with the transformation from the root to the board frame in [x,y,z,qx,qy,qz,qw]", required=true, minimum = -2, maximum = 2, minItems = 7, maxItems = 7})
})

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    param.get("FT_sensor_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    -- "forearm"
    -- "tcp_frame"
    --Add all frames that are required by the task specification
})
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))
FT_sensor_frame = robot.getFrame(param.get("FT_sensor_frame"))

-- ========================================= PARAMETERS ===================================

small_number = 0.000000000001
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))
eqradius = constant(param.get("eq_r"))

cable_tens = constant(param.get("cable_tens"))
force_threshold = constant(param.get("force_threshold"))
torque_threshold = constant(param.get("torque_threshold"))
tool_weight = constant(param.get("tool_weight"))
k_x = constant(param.get("k_x"))
z_down = constant(param.get("z_down"))

previous_to_current_fixture = param.get("previous_to_current_fixture")
previous_to_current_fixture_x = constant(previous_to_current_fixture[1])
previous_to_current_fixture_y = constant(previous_to_current_fixture[2])
previous_to_current_fixture_z = constant(previous_to_current_fixture[3])

tool_COG = param.get("tool_COG")
tool_COG_x = constant(tool_COG[1])
tool_COG_y = constant(tool_COG[2])
tool_COG_z = constant(tool_COG[3])

T_root_board_p = param.get("T_root_board")
T_root_board_x = constant(T_root_board_p[1])
T_root_board_y = constant(T_root_board_p[2])
T_root_board_z = constant(T_root_board_p[3])
T_root_board_q_i = constant(T_root_board_p[4])
T_root_board_q_j = constant(T_root_board_p[5])
T_root_board_q_k = constant(T_root_board_p[6])
T_root_board_q_real = constant(T_root_board_p[7])

T_root_board_quat = quaternion(T_root_board_q_real, vector(T_root_board_q_i, T_root_board_q_j, T_root_board_q_k))
T_root_board_R = toRot(T_root_board_quat)
T_root_board_P = vector(T_root_board_x, T_root_board_y, T_root_board_z)
T_root_board = frame(T_root_board_R, T_root_board_P)

-- ========================================= Variables coming from topic input handlers ===================================
sensed_wrench   = ctx:createInputChannelWrench("wrench_input")

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
print("CHAAAAAAAA LOK 1")
d_g = vector(0,0,-1)
FT_sensor_frame_to_cog = frame(vector(tool_COG_x, tool_COG_y, tool_COG_z))

virtual_wrench_wrt_base_frame = wrench(d_g*tool_weight, cross(origin(FT_sensor_frame_to_cog),d_g*tool_weight))

wrench_cog_ftframe = transform(rotation(inv(FT_sensor_frame)), wrench(d_g*tool_weight, vector(0,0,0)))
wrench_FT_frame = ref_point(wrench_cog_ftframe, -origin(FT_sensor_frame_to_cog))

wrench_dead_zone = wrench(vector(Fx_dead_zone,Fy_dead_zone,Fz_dead_zone),vector(Tx_dead_zone,Ty_dead_zone,Tz_dead_zone)) - wrench_FT_frame - virtual_wrench_wrt_base_frame

-- ======================================== FRAMES ========================================
print("CHAAAAAAAA LOK 2")

angle = atan2(previous_to_current_fixture_y, previous_to_current_fixture_x+small_number) + math.pi -- +math.pi to align x looking to previous fixture
z_vec = vector(0,0,1)
-- TODO: ERROR: This should not be wrt the base frame. Change it for any orientation of the board
fix_rotation = rotVec(z_vec, angle)

-- =============================== INITIAL POSE ==============================
print("CHAAAAAAAA LOK 3")
startpose = initial_value(time, task_frame)
startpos  = origin(startpose)
startrot  = rotation(startpose)
-- start_pose_diff  = inv(startpose)*task_frame

endpos = origin(startpose * frame(vector(0,0,-z_down)))
endrot = rotation(T_root_board * frame(fix_rotation))

-- ========================================== Imports utils_ts ============================
print("CHAAAAAAAA LOK 5")
-- The following is done because utils_ts is a file of the library and not of the application ROS2 package.
local script_dir = debug.getinfo(1, "S").source:match("@(.*)/")
package.path = script_dir .. "/utilities/?.lua;" .. package.path  -- Add it to package.path
local utils_ts = require("utils_ts")

-- =========================== VELOCITY PROFILE ============================================
print("CHAAAAAAAA LOK 6")

diff_rot                = cached(  getRotVec( inv(startrot)*endrot )) -- eq. axis of rotation for rotation from start to end:w
diff_rot, angle_delta         = utils_ts.normalize( diff_rot )
-- plan trapezoidal motion profile in function of time:
mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
mp:addOutput(constant(0), z_down, maxvel, maxacc)
mp:addOutput(constant(0), angle_delta*eqradius, maxvel, maxacc)

progress_distance  = get_output_profile(mp,0)            -- progression in distance
progress_distance_rot  = get_output_profile(mp,1)/eqradius   -- progression in distance_rot (i.e. rot*eqradius)

-- =========================== TARGET POSE ============================================
print("CHAAAAAAAA LOK 7")
targetrot = startrot*rotVec(diff_rot,progress_distance_rot)
target_z = -progress_distance


Constraint{
    context = ctx,
    name    = "follow_orientation",
    expr    = inv(targetrot)*rotation(task_frame),
    K       = 4,
    weight  = 1,
    priority= 2
};

task_frame_wrt_board = inv(T_root_board)*task_frame
initial_z = coord_z(origin(initial_value(time, task_frame_wrt_board)))
Constraint{
    context = ctx,
    name    = "follow_position",
    expr    = coord_z(origin(task_frame_wrt_board)),
    target  = initial_z + target_z,
    K       = 4,
    weight  = 1,
    priority= 2
};


-- =========================== FORCES =============================================
print("CHAAAAAAAA LOK 8")
task_frame_tension = frame(endrot, origin(task_frame))
task_frame_inst = inv(make_constant(task_frame_tension))*task_frame_tension
rot_vec = getRotVec(rotation(task_frame_inst))

-- =============================== TRANSLATE FT TO TASK_FRAME ==============================
print("CHAAAAAAAA LOK 9")
wrench_task_frame = ref_point(transform(rotation(inv(task_frame_tension)*FT_sensor_frame), wrench_dead_zone) , -origin(inv(task_frame_tension)*FT_sensor_frame))

Fx = coord_x(force(wrench_task_frame))
Fy = coord_y(force(wrench_task_frame))
Fz = coord_z(force(wrench_task_frame))
Tx = coord_x(torque(wrench_task_frame))
Ty = coord_y(torque(wrench_task_frame))
Tz = coord_z(torque(wrench_task_frame))


-- Constraint{
--     context = ctx,
--     name    = "constant_y",
--     expr    = coord_y(origin(start_pose_diff)),
--     K       = 4,
--     weight  = 1,
--     priority= 2
-- };

Constraint{
	context=ctx,
	name="keep_cable_tension",
	model = -k_x*coord_x(origin(task_frame_inst)),
    meas = Fx,
	target = cable_tens,
	K = 4,
	priority = 2,
	weight = 1
};

Constraint{
    context = ctx,
    name    = "tangential_setpoint",
    expr    = coord_y(origin(task_frame_inst)) - 0*time,
    K       = 0,
    weight  = 1,
    priority= 2
};

print("CHAAAAAAAA LOK 9")
--Error calculation
local rot_vec = getRotVec(inv(rotation(task_frame))*endrot)
local diff_rot_error
local angle_error
diff_rot_error, angle_error = utils_ts.normalize(rot_vec)

local error_pos = abs(coord_z(origin(task_frame_wrt_board)) - (initial_z - z_down)) -- The error in the z direction
local error_orient = angle_error 
local error = conditional(error_pos-(1/1000), constant(0), constant(1))*conditional(error_orient-(0.5*math.pi/180), constant(0), constant(1)) -- Returns zero if the error is too big

Monitor{
    context=ctx,
    name='finish_after_motion',
    upper = 0.8, -- The error, that comes from a conditional, is binary (0 or 1)
    actionname = "exit",
    expr=error
};

-- Monitor{
--     context=ctx,
--     name='debug_monitor',
--     upper = 4, 
--     actionname = "debug",
--     expr=time
-- };

-- ============================== OUTPUT THROUGH PORTS===================================
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))

roll_tf,pitch_tf,yaw_tf = getRPY(rotation(task_frame))
ctx:setOutputExpression("roll_tf",roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf",yaw_tf)

ctx:setOutputExpression("Fx"      ,Fx)
ctx:setOutputExpression("Fy"      ,Fy)
ctx:setOutputExpression("Fz"      ,Fz)
ctx:setOutputExpression("Tx"      ,Tx)
ctx:setOutputExpression("Ty"      ,Ty)
ctx:setOutputExpression("Tz"      ,Tz)