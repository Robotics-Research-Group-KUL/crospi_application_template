require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")

reqs = require("task_requirements")
-- TODO
task_description = " fgffv "

-- ========================================= FUNCTIONS ===================================
function dead_zone(signal,dead_val)
    signal_dead_zone = conditional(abs(signal)-dead_val, signal + conditional(signal, -dead_val, dead_val), constant(0))
    return signal_dead_zone
end

-- ========================================= PARAMETERS ===================================
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="cable_tens", description="Desired Cable Tension [N]", default = 5.0, required=true, maximum = 15}),
    reqs.params.scalar({name="force_threshold", description="Force dead zone [N]", default = 0.5, required=false}),
    reqs.params.scalar({name="torque_threshold", description="Torque dead zone [Nm]", default = 0.05, required=false}),
    reqs.params.array({name="tool_COG", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, 
                            description="Array with the center of gravity of the tool w.r.t FT_sensor_frame [m]", required=true, minItems = 3, maxItems = 3}),
    reqs.params.scalar({name="tool_weight", description="Weight of the tool attached to the end-effector [N]", default = 0.0, required=true}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=true}),
    reqs.params.string({name="FT_sensor_frame", description="Name of frame where the forces and torques a measured", default = "FT_sensor_frame", required=true}),
    reqs.params.scalar({name="k_x", description="Stiffness in the cable pulling direction", default=2700,  required=true, minimum = 1500, maximum = 4000})
})

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    param.get("FT_sensor_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    -- "forearm"
    -- "tcp_frame"
    -- Add all frames that are required by the task specification
})

robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))
FT_sensor_frame = robot.getFrame(param.get("FT_sensor_frame"))
 
-- ========================================= PARAMETERS ===================================
cable_tens          = constant(param.get("cable_tens"))
force_threshold     = constant(param.get("force_threshold"))
torque_threshold    = constant(param.get("torque_threshold"))
tool_COG            = param.get("tool_COG")
tool_weight         = constant(param.get("tool_weight"))
k_x                 = param.get("k_x")

tool_COG_x = constant(tool_COG[1])
tool_COG_y = constant(tool_COG[2])
tool_COG_z = constant(tool_COG[3])

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
d_g = vector(0,0,-1) -- wrt to the base frame
FT_sensor_frame_to_cog = frame(vector(tool_COG_x, tool_COG_y, tool_COG_z))

-- This is the wrench removed by the taring and that needs to be compensated (Assuming it was tared aligned with the gravity vector)
virtual_wrench_wrt_base_frame = wrench(d_g*tool_weight, cross(origin(FT_sensor_frame_to_cog),d_g*tool_weight))
virtual_wrench_wrt_FT_frame = transform(rotation(FT_sensor_frame), virtual_wrench_wrt_base_frame)

-- This is the wrench caused by the tool weight that needs to be compensated in the FT_sensor_frame
-- Rotate the wrench to the FT_sensor_frame orientation
wrench_cog_ftframe = transform(rotation(inv(FT_sensor_frame)), wrench(d_g*tool_weight, vector(0,0,0)))

-- Translate the wrench to the FT_sensor_frame
wrench_FT_frame = ref_point(wrench_cog_ftframe, origin(inv(FT_sensor_frame_to_cog)))
wrench_dead_zone = wrench(vector(Fx_dead_zone,Fy_dead_zone,Fz_dead_zone),vector(Tx_dead_zone,Ty_dead_zone,Tz_dead_zone)) - wrench_FT_frame + virtual_wrench_wrt_FT_frame

-- =============================== TRANSLATE FT TO TASK_FRAME ==============================
wrench_task_frame   = ref_point(transform(rotation(inv(task_frame)*FT_sensor_frame), wrench_dead_zone) , -origin(inv(task_frame)*FT_sensor_frame))

Fx = coord_x(force(wrench_task_frame))
Fy = coord_y(force(wrench_task_frame))
Fz = coord_z(force(wrench_task_frame))
Tx = coord_x(torque(wrench_task_frame))
Ty = coord_y(torque(wrench_task_frame))
Tz = coord_z(torque(wrench_task_frame))

-- =============================== INSTANTANEOUS FRAME ==============================
task_frame_inst = inv(make_constant(task_frame))*task_frame

-- =============================== INITIAL VALUES ==============================
startpose = initial_value(time, task_frame)
start_pose_diff  = inv(startpose)*task_frame

-- =============================== CONSTRAINT SPECIFICATION ==============================
Constraint{
	context=ctx,
	name="follow_force_x",
	model = -k_x*coord_x(origin(task_frame_inst)),
	meas = Fx,
	target = -cable_tens,
	K = 4,
	priority = 2,
	weight = 1
};

Constraint{
    context = ctx,
    name    = "constant_y",
    expr    = coord_y(origin(start_pose_diff)),
    K       = 4,
    weight  = 1,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "constant_z",
    expr    = coord_z(origin(start_pose_diff)),
    K       = 4,
    weight  = 1,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "constant_orientation",
    expr    = rotation(start_pose_diff),
    K       = 4,
    weight  = 1,
    priority= 2
};

-- Max tensioning velocity
Constraint{
    context = ctx,
    name    = "x_velocity",
    expr    = coord_x(origin(task_frame_inst)),
	target_upper = maxvel*time,
	target_lower = -maxvel*time,
    K       = 0,
    weight  = 10,
    priority= 1
};

Monitor{
    context=ctx,
    name='finish_force',
    upper=0.8,
    actionname='exit',
    expr=abs(Fx)/cable_tens
};

roll_tf, pitch_tf, yaw_tf = getRPY(rotation(task_frame))

ctx:setOutputExpression("x_tf"		,coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf"		,coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf"		,coord_z(origin(task_frame)))
ctx:setOutputExpression("roll_tf"	,roll_tf)
ctx:setOutputExpression("pitch_tf"  ,pitch_tf)
ctx:setOutputExpression("yaw_tf"	,yaw_tf)

ctx:setOutputExpression("Fx"      ,Fx)
ctx:setOutputExpression("Fy"      ,Fy)
ctx:setOutputExpression("Fz"      ,Fz)
ctx:setOutputExpression("Tx"      ,Tx)
ctx:setOutputExpression("Ty"      ,Ty)
ctx:setOutputExpression("Tz"      ,Tz)