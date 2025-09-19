require("context")
require("geometric")
-- worldmodel=require("worldmodel")

require("math")
reqs = require("task_requirements")

task_description = "This task specification allows to move the position of the end effector in cartesian space relative to the initial pose"

-- ========================================= FUNCTIONS ===================================
function dead_zone(signal,dead_val)
    signal_dead_zone = conditional(abs(signal)-dead_val, signal + conditional(signal, -dead_val, dead_val), constant(0))
    return signal_dead_zone
end


-- ========================================= PARAMETERS ===================================

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration [m/s^2]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=false}),
    reqs.params.array({name="delta_euler", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, description="3D array of euler angles [rad] that the robot will move w.r.t. the starting orientation following RPY convention w.r.t the robot base", required=true,minimum = -6.28, maximum=6.28,minItems = 3, maxItems = 3}),
    reqs.params.scalar({name="contact_force", description="Contact force [N]", default = 0.0, required=true, maximum = 15}),
    reqs.params.scalar({name="force_threshold", description="Force dead zone [N]", default = 0.5, required=false}),
    reqs.params.scalar({name="torque_threshold", description="Torque dead zone [Nm]", default = 0.05, required=false}),
    reqs.params.array({name="tool_COG", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, 
                            description="Array with the center of gravity of the tool w.r.t FT_sensor_frame [m]", required=true, minItems = 3, maxItems = 3}),
    reqs.params.scalar({name="tool_weight", description="Weight of the tool attached to the end-effector [N]", default = 0.0, required=true}),
    reqs.params.string({name="FT_sensor_frame", description="Name of frame where the forces and torques a measured", default = "FT_sensor_frame", required=true}),
    reqs.params.array({name="K_F", type=reqs.array_types.number, default={2700, 2700, 2700},
                            description="Array with the diagonal elements of the translational stiffness matrix [N/m]", required=true, minimum = 0.0, minItems = 3, maxItems = 3}),
    
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
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))

delta_euler = param.get("delta_euler")
delta_roll   = constant(delta_euler[1])
delta_pitch   = constant(delta_euler[2])
delta_yaw   = constant(delta_euler[3])

contact_force       = constant(param.get("contact_force"))
force_threshold     = constant(param.get("force_threshold"))
torque_threshold    = constant(param.get("torque_threshold"))
tool_COG            = param.get("tool_COG")
tool_weight         = constant(param.get("tool_weight"))
K_F                 = param.get("K_F")

tool_COG_x = constant(tool_COG[1])
tool_COG_y = constant(tool_COG[2])
tool_COG_z = constant(tool_COG[3])

K_F_x = constant(K_F[1])
K_F_y = constant(K_F[2])
K_F_z = constant(K_F[3])

-- =============================== INITIAL POSE ==============================

startpose = initial_value(time, task_frame)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- =============================== END POSE ==============================

end_frame = startpose*frame(rot_x(delta_roll)*rot_y(delta_pitch)*rot_z(delta_yaw),vector(0,0,0))
endpos = origin(end_frame)
endrot = rotation(end_frame)


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
wrench_task_frame   = ref_point(transform(rotation(inv(task_frame)*FT_sensor_frame), wrench_dead_zone) , -origin(inv(task_frame)*FT_sensor_frame))

Fx = coord_x(force(wrench_task_frame))
Fy = coord_y(force(wrench_task_frame))
Fz = coord_z(force(wrench_task_frame))
Tx = coord_x(torque(wrench_task_frame))
Ty = coord_y(torque(wrench_task_frame))
Tz = coord_z(torque(wrench_task_frame))

-- =============================== INSTANTANEOUS FRAME ==============================
task_frame_inst = inv(make_constant(task_frame))*task_frame

-- ========================================== Imports utils_ts ============================
-- The following is done because utils_ts is a file of the library and not of the application ROS2 package.
local script_dir = debug.getinfo(1, "S").source:match("@(.*)/")
package.path = script_dir .. "/utilities/?.lua;" .. package.path  -- Add it to package.path
local utils_ts = require("utils_ts")

-- =========================== VELOCITY PROFILE ============================================
progress_variable = Variable{context = ctx, name ='path_coordinate', vartype = 'feature', initial = 0.0}

eps=constant(1E-14)

diff_rot                = cached(  getRotVec( inv(startrot)*endrot )) -- eq. axis of rotation for rotation from start to end:w
diff_rot, angle         = utils_ts.normalize( diff_rot )

r_inst = angle*progress_variable
-- =========================== VELOCITY PROFILE ============================================
d_time = constant(0)
mt=constant(1)

A = conditional(time-d_time,constant(1),constant(0))

s_initial = constant(0)
s_final = constant(1)
s_vel_motion_profile  = utils_ts.trap_velprofile( maxvel , maxacc , constant(0.0) , constant(1.0),  progress_variable)
s_velocity = conditional( time-d_time , s_vel_motion_profile , constant(0) )

Constraint{
	context=ctx,
	name = "vel_prof",
	expr = progress_variable - s_velocity*time,
	weight = 10,
	priority = 2,
	K = 0
};

Constraint{
	context=ctx,
	name = "reaching_vel_max",
	expr = progress_variable - maxvel*time,
	weight = 0.001,
	priority = 2,
	K = 0
};

Constraint{
	context=ctx,
	name = "s_min",
	expr = progress_variable,
	target_lower = s_initial,
	weight = 20,
	priority = 2,
	K = 4
};

Constraint{
	context=ctx,
	name = "s_max",
	expr = progress_variable,
	target_upper = s_final,
	weight = 20,
	priority = 2,
	K = 4
};

-- =========================== TARGET POSE ============================================
targetpos = startpos
targetrot = startrot*rotVec(diff_rot,r_inst)

target    = frame(targetrot,targetpos)

-- ========================== CONSTRAINT SPECIFICATION =================================
-- Forces Position
Constraint{
	context=ctx,
	name="follow_force_x",
	model = -K_F_x*coord_x(origin(task_frame_inst)),
	meas = Fx,
	target = 0.0,
	K = constant(4),
	priority = 2,
	weight = constant(10),
};

Constraint{
	context=ctx,
	name="follow_force_y",
	model = -K_F_y*coord_y(origin(task_frame_inst)),
	meas = Fy,
	target = 0.0,
	K = constant(4),
	priority = 2,
	weight = constant(10),
};

Constraint{
	context=ctx,
	name="follow_force_z",
	model = -K_F_z*coord_z(origin(task_frame_inst)),
	meas = Fz,
	target = contact_force,
	K = constant(4),
	priority = 2,
	weight = constant(10),
};


Constraint{
    context = ctx,
    name    = "follow_orientation",
    expr    = inv(rotation(target))*rotation(task_frame),
    K       = 3,
    weight  = 10,
    priority= 2
}


-- ========================= Velocity contraint joint ==================
base_joint = ctx:getScalarExpr(robot.robot_joints[1])
Constraint{
    context = ctx,
    name    = "low_vel_joint",
    expr    = base_joint - 0*time,
    K       = 0,
    weight  = 4,
    priority= 2
}

-- =========================== MONITOR ============================================
err = (s_final-progress_variable)
Monitor{
        context=ctx,
        name='finish_after_motion',
        lower=0.000,
        actionname='exit',
        expr=err
}

-- Monitor{
--     context=ctx,
--     name='portevent_test',
--     upper=0.0,
--     actionname='portevent',
--     argument = "test_event",
--     expr=time-0.5
-- }

-- Monitor{
--     context=ctx,
--     name='event_test',
--     upper=0.0,
--     actionname='event',
--     argument = "test_event",
--     expr=time-0.5
-- }


ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))

-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(task_frame))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)