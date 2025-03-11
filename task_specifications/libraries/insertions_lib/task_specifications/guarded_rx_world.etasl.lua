require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")

reqs = require("task_requirements")
-- TODO
task_description = ""

-- ========================================= FUNCTIONS ===================================
function dead_zone(signal,dead_val)
    signal_dead_zone = conditional(abs(signal)-dead_val, signal + conditional(signal, -dead_val, dead_val), constant(0))
    return signal_dead_zone
end

-- ========================================= PARAMETERS ===================================
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="contact_force", description="Contact force [N]", default = 0.0, required=true, maximum = 15}),
    reqs.params.scalar({name="force_threshold", description="Force dead zone [N]", default = 0.5, required=false}),
    reqs.params.scalar({name="torque_threshold", description="Torque dead zone [Nm]", default = 0.05, required=false}),
    reqs.params.array({name="tool_COG", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, 
                            description="Array with the center of gravity of the tool w.r.t FT_sensor_frame [m]", required=true, minItems = 3, maxItems = 3}),
    reqs.params.array({name="R_tf_2_insframe", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 1.0}, 
                            description="Quaternion with the rotation matrix from the task_frame to the insertion_frame in [qx,qy,qz,qw]", required=true, minItems = 4, maxItems = 4}),
    reqs.params.scalar({name="tool_weight", description="Weight of the tool attached to the end-effector [N]", default = 0.0, required=true}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=true}),
    reqs.params.string({name="FT_sensor_frame", description="Name of frame where the forces and torques a measured", default = "FT_sensor_frame", required=true}),
    reqs.params.array({name="K_F", type=reqs.array_types.number, default={2700, 2700, 2700},
                            description="Array with the diagonal elements of the translational stiffness matrix [N/m]", required=true, minimum = 0.0, minItems = 3, maxItems = 3}),
    reqs.params.array({name="K_T", type=reqs.array_types.number, default={200, 200, 200},
                            description="Array with the diagonal elements of the rotational stiffness matrix [Nm/rad]", required=true, minimum = 0.0, minItems = 3, maxItems = 3})
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
maxvel              = constant(param.get("maxvel"))
contact_force       = constant(param.get("contact_force"))
force_threshold     = constant(param.get("force_threshold"))
torque_threshold    = constant(param.get("torque_threshold"))
tool_COG            = param.get("tool_COG")
tool_weight         = constant(param.get("tool_weight"))
K_F                 = param.get("K_F")
K_T                 = param.get("K_T")
R_tf_2_insframe     = param.get("R_tf_2_insframe")

tool_COG_x = constant(tool_COG[1])
tool_COG_y = constant(tool_COG[2])
tool_COG_z = constant(tool_COG[3])

K_F_x = constant(K_F[1])
K_F_y = constant(K_F[2])
K_F_z = constant(K_F[3])

K_T_x = constant(K_T[1])
K_T_y = constant(K_T[2])
K_T_z = constant(K_T[3])

q_i     = constant(R_tf_2_insframe[1])
q_j     = constant(R_tf_2_insframe[2])
q_k     = constant(R_tf_2_insframe[3])
q_real  = constant(R_tf_2_insframe[4])

-- compute orientation from quaternion
quat_tf_2_insframe = quaternion(q_real,vector(q_i,q_j,q_k))
tf_2_insframe = frame(toRot(quat_tf_2_insframe), vector(0,0,0))

-- ========================================= Variables coming from topic input handlers ===================================
sensed_wrench   = ctx:createInputChannelWrench("wrench_input")

-- =============================== TRANSFORM WRENCH TO TASK FRAME ==============================
Fx = coord_x(force(sensed_wrench))
Fy = coord_y(force(sensed_wrench))
Fz = coord_z(force(sensed_wrench))
Tx = coord_x(torque(sensed_wrench))
Ty = coord_y(torque(sensed_wrench))
Tz = coord_z(torque(sensed_wrench))

Fx_dead_zone = dead_zone(Fx,force_threshold)
Fy_dead_zone = dead_zone(Fy,force_threshold)
Fz_dead_zone = dead_zone(Fz,force_threshold)
Tx_dead_zone = dead_zone(Tx,torque_threshold)
Ty_dead_zone = dead_zone(Ty,torque_threshold)
Tz_dead_zone = dead_zone(Tz,torque_threshold)

-- =============================== GRAVITY COMPENSATION ==============================
d_g = vector(0,0,-1)
FT_sensor_frame_to_cog = frame(vector(tool_COG_x, tool_COG_y, tool_COG_z))

virtual_wrench = wrench(d_g*tool_weight, cross(origin(FT_sensor_frame_to_cog),d_g*tool_weight))

wrench_cog_ftframe = transform(rotation(inv(FT_sensor_frame)), wrench(d_g*tool_weight, vector(0,0,0)))
wrench_FT_frame = ref_point(wrench_cog_ftframe, -origin(FT_sensor_frame_to_cog))

wrench_dead_zone = wrench(vector(Fx_dead_zone,Fy_dead_zone,Fz_dead_zone),vector(Tx_dead_zone,Ty_dead_zone,Tz_dead_zone)) - wrench_FT_frame - virtual_wrench

-- =============================== TRANSLATE FT TO TASK_FRAME ==============================
task_frame = task_frame*tf_2_insframe

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
    context = ctx,
    name    = "follow_path",
    expr    = origin(start_pose_diff),
    K       = 4,
    weight  = 1000,
    priority= 2
};

contact_torque = 0.15

Constraint{
	context=ctx,
	name="follow_torque_x",
	model = -K_Tx*coord_x(getRotVec(rotation(task_frame_inst))),
	meas = Tx,
	target = -contact_torque,
	K = constant(4),
	priority = 2,
	weight = constant(1),
};

Constraint{
    context = ctx,
    name    = "constant_orientation_y",
    expr    = coord_y(rotation(start_pose_diff)),
    K       = 4,
    weight  = 1,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "constant_orientation_z",
    expr    = coord_z(rotation(start_pose_diff)),
    K       = 4,
    weight  = 1,
    priority= 2
};

time_start = conditional(time-1,2,0)

monitor_F = Monitor{context=ctx, 
					name='finish_torque',
					upper=0.8,
					actionname='exit', 
					expr=time_start*abs(Tx)/contact_torque
				};

roll_tf, pitch_tf, yaw_tf = getRPY(rotation(task_frame))

-- ctx:setOutputExpression("x_tf"		,coord_x(origin(task_frame)))
-- ctx:setOutputExpression("y_tf"		,coord_y(origin(task_frame)))
-- ctx:setOutputExpression("z_tf"		,coord_z(origin(task_frame)))
-- ctx:setOutputExpression("roll_tf"	,roll_tf)
-- ctx:setOutputExpression("pitch_tf"  ,pitch_tf)
-- ctx:setOutputExpression("yaw_tf"	,yaw_tf)

-- ctx:setOutputExpression("Fx"      ,Fx)
-- ctx:setOutputExpression("Fy"      ,Fy)
-- ctx:setOutputExpression("Fz"      ,Fz)
-- ctx:setOutputExpression("Tx"      ,Tx)
-- ctx:setOutputExpression("Ty"      ,Ty)
-- ctx:setOutputExpression("Tz"      ,Tz)

-- ctx:setOutputExpression("tcp_frame"   , tcp_frame)
-- ctx:setOutputExpression("task_frame"  ,task_frame)



-- contact_force   = constant(createScalarParameter("contact_force", 0.0))
-- max_v   = constant(createScalarParameter("max_v"  ,0.02))
-- K_Fx = constant(createScalarParameter("K_Fx"  ,2700)) -- compliance in x-axis
-- K_Fy = constant(createScalarParameter("K_Fy"  ,2700)) -- compliance in y-axis
-- K_Fz = constant(createScalarParameter("K_Fz"  ,2700)) -- compliance in z-axis
-- K_Tx = constant(createScalarParameter("K_Tx"  ,200))

-- tool_weight = constant(createScalarParameter("tool_weight"  ,11.10748))
-- COG_x = constant(createScalarParameter("COG_x"  ,-0.0031757))
-- COG_y = constant(createScalarParameter("COG_y"  ,0.001138889))
-- COG_z = constant(createScalarParameter("COG_z"  ,0.0667538))

-- -- TCP to TF transform
-- x_coordinate   = constant(createScalarParameter("x",0.0, "Absolute X coordinate"))
-- y_coordinate   = constant(createScalarParameter("y",0.0, "Absolute Y coordinate"))
-- z_coordinate   = constant(createScalarParameter("z",0.0, "Absolute Z coordinate"))
-- q_real  = constant(createScalarParameter("q_real",1.0, "Real part of the quaternion"))
-- q_i     = constant(createScalarParameter("q_i",0.0, "i part of the quaternion"))
-- q_j     = constant(createScalarParameter("q_j",0.0, "j part of the quaternion"))
-- q_k     = constant(createScalarParameter("q_k",0.0, "k part of the quaternion"))

-- quat = quaternion(q_real,vector(q_i,q_j,q_k))
-- transform_R = toRot(quat)
-- transform_P = vector(x_coordinate,y_coordinate,z_coordinate) 
-- tcp_2_tf =frame(transform_R, transform_P)