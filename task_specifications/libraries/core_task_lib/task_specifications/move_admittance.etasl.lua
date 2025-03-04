require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
require("etasl_parameters")

set_task_description("This task specification allows to control the robot with an admittance controller using force/torque sensor data.")

-- ========================================= PARAMETERS ===================================
K_t   = constant(createScalarParameter("K_t",2000, "Translational stiffness [N/m]"))
K_r   = constant(createScalarParameter("K_r",200, "Rotational stiffness [Nm/rad]"))
sensed_wrench   = ctx:createInputChannelWrench("wrench_input")
-- sensed_wrench = wrench(vector(0,0,-5),vector(0,0,0))

function dead_zone(signal,dead_val)
    signal_dead_zone = conditional(abs(signal)-dead_val, signal + conditional(signal, -dead_val, dead_val), constant(0))
    return signal_dead_zone
end

-- ======================================== FRAMES ========================================

task_frame = ee
-- =============================== INSTANTANEOUS FRAME ==============================

task_frame_inst = inv(make_constant(task_frame))*task_frame
-- =============================== TRANSFORM WRENCH TO TASK FRAME ==============================

Fx = coord_x(force(sensed_wrench))
Fy = coord_y(force(sensed_wrench))
Fz = coord_z(force(sensed_wrench))
Tx = coord_x(torque(sensed_wrench))
Ty = coord_y(torque(sensed_wrench))
Tz = coord_z(torque(sensed_wrench))

force_th = constant(0.5)
torque_th = constant(0.05)

Fx_dead_zone = dead_zone(Fx,force_th)
Fy_dead_zone = dead_zone(Fy,force_th)
Fz_dead_zone = dead_zone(Fz,force_th)
Tx_dead_zone = dead_zone(Tx,torque_th)
Ty_dead_zone = dead_zone(Ty,torque_th)
Tz_dead_zone = dead_zone(Tz,torque_th)

-- =============================== GRAVITY COMPENSATION ==============================

d_g = vector(0,0,-1)
tool_weight = 11.10748 --kg
FT_sensor_frame_to_cog = frame(vector(-0.0031757,0.001138889,0.0667538))

virtual_wrench = wrench(d_g*tool_weight, cross(origin(FT_sensor_frame_to_cog),d_g*tool_weight))

wrench_cog_ftframe = transform(rotation(inv(FT_sensor_frame)), wrench(d_g*tool_weight, vector(0,0,0)))
wrench_FT_frame = ref_point(wrench_cog_ftframe, -origin(FT_sensor_frame_to_cog))

wrench_dead_zone = wrench(vector(Fx_dead_zone,Fy_dead_zone,Fz_dead_zone),vector(Tx_dead_zone,Ty_dead_zone,Tz_dead_zone)) - wrench_FT_frame - virtual_wrench

wrench_task_frame   = ref_point(transform(rotation(inv(task_frame)*FT_sensor_frame),wrench_dead_zone) , -origin(inv(task_frame)*FT_sensor_frame))

Fx = coord_x(force(wrench_task_frame))
Fy = coord_y(force(wrench_task_frame))
Fz = coord_z(force(wrench_task_frame))
Tx = coord_x(torque(wrench_task_frame))
Ty = coord_y(torque(wrench_task_frame))
Tz = coord_z(torque(wrench_task_frame))

-- =============================== ADMITTANCE CONTROLLER ==============================
-- Force constraints
Constraint{
	context=ctx,
	name="follow_force_x",
	model = -K_t*coord_x(origin(task_frame_inst)),
	meas = Fx,
	target = 0,
	K = constant(4),
	priority = 2,
	weight = constant(1),
};

Constraint{
	context=ctx,
	name="follow_force_y",
	model = -K_t*coord_y(origin(task_frame_inst)),
	meas = Fy,
	target = 0,
	K = constant(4),
	priority = 2,
	weight = constant(1),
};

Constraint{
	context=ctx,
	name="follow_force_z",
	model = -K_t*coord_z(origin(task_frame_inst)),
	meas = Fz,
	target = 0,
	K = constant(4),
	priority = 2,
	weight = constant(1),
};

-- Torque constraints
Constraint{
    context=ctx,
    name="follow_torque_x",
    model = -K_r*coord_x(getRotVec(rotation(task_frame_inst))),
    meas = Tx,
    target = 0,
    K = constant(4),
    priority = 2,
    weight = constant(1),
};

Constraint{
    context=ctx,
    name="follow_torque_y",
    model = -K_r*coord_y(getRotVec(rotation(task_frame_inst))),
    meas = Ty,
    target = 0,
    K = constant(4),
    priority = 2,
    weight = constant(1),
};

Constraint{
    context=ctx,
    name="follow_torque_z",
    model = -K_r*coord_z(getRotVec(rotation(task_frame_inst))),
    meas = Tz,
    target = 0,
    K = constant(4),
    priority = 2,
    weight = constant(1),
};

-- -- Constant orientation
-- intial_rot=initial_value(time,rotation(task_frame))
-- Constraint {
--     context         = ctx,
--     name            = "keep_rot",
--     expr            = rotation(task_frame)*inv(intial_rot),
--     weight          = 1,
--     priority        = 2,
--     K               = 4
-- };


ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tcp",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tcp",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tcp",coord_z(origin(task_frame)))
ctx:setOutputExpression("tf",task_frame)




-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(tf)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(tf)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(tf)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(tf))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)