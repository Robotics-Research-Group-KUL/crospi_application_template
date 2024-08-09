-- ==============================================================================
-- Author: Santiago Iregui
-- email: <santiago.iregui@kuleuven.be>
-- Task specification of assistance for articulated objects
-- KU Leuven 2024
-- ==============================================================================
require("context")
require("geometric")
-- worldmodel=require("worldmodel")

adaptive_task_frame = frame(vector(0,0,0))
FT_sensor_frame = tcp_frame

translation_adm = true
rotation_adm = true



use_adaptive_task_frame = true
control_at_tcp = true

-- max_v   = ctx:createInputChannelScalar("max_v"  ,0.10)
-- max_omega   = ctx:createInputChannelScalar("max_omega"  ,11*3.1416/180)

max_v = 0.10
max_omega = 11*3.1416/180

Fx_raw   = ctx:createInputChannelScalar("Fx_raw"  , 0.0)
Fy_raw   = ctx:createInputChannelScalar("Fy_raw"  , 0.0)
Fz_raw   = ctx:createInputChannelScalar("Fz_raw"  , 0.0)
Tx_raw   = ctx:createInputChannelScalar("Tx_raw"  , 0.0)
Ty_raw   = ctx:createInputChannelScalar("Ty_raw"  , 0.0)
Tz_raw   = ctx:createInputChannelScalar("Tz_raw"  , 0.0)

alpha_sel   = ctx:createInputChannelScalar("alpha_sel"  , 1)
alpha_prism   = ctx:createInputChannelScalar("alpha_prism"  , 1)
alpha_rev   = ctx:createInputChannelScalar("alpha_rev"  , 1)

joystick_input_raw   = ctx:createInputChannelTwist("joystick_input")
Vx_trans   = coord_x(transvel(joystick_input_raw))/constant(0.69)
Vy_trans   = coord_y(transvel(joystick_input_raw))/constant(0.69)
Vz_trans   = coord_z(transvel(joystick_input_raw))/constant(0.69)
Vx_rot   = coord_x(rotvel(joystick_input_raw))/constant(0.69)
Vy_rot   = coord_y(rotvel(joystick_input_raw))/constant(0.69)
Vz_rot   = coord_z(rotvel(joystick_input_raw))/constant(0.69)


-- Vx_trans   = ctx:createInputChannelScalar("Vx_trans"  , 0.0)
-- Vy_trans   = ctx:createInputChannelScalar("Vy_trans"  , 0.0)
-- Vz_trans   = ctx:createInputChannelScalar("Vz_trans"  , 0.0)
-- Vx_rot   = ctx:createInputChannelScalar("Vx_rot"  , 0.0)
-- Vy_rot   = ctx:createInputChannelScalar("Vy_rot"  , 0.0)
-- Vz_rot   = ctx:createInputChannelScalar("Vz_rot"  , 0.0)



-- ========================================= PARAMETERS ===================================


C_Fx = constant(0.0001875*2) -- compliance in x-axis linear
C_Fy = constant(0.0001875*2) -- compliance in y-axis linear
C_Fz = constant(0.0001875*2) -- compliance in z-axis linear
C_Tx = constant(0.0475*0.5) -- compliance about x-axis angular
C_Ty = constant(0.0475*0.5) -- compliance about y-axis angular
C_Tz = constant(0.0475*0.5) -- compliance about z-axis angular


K_F = constant(4) -- control gain linear
K_T = constant(4) -- control gain angular

force_threshold =  constant(0.5)
torque_threshold = constant(0.03)

if use_adaptive_task_frame then
	tf = tcp_frame*adaptive_task_frame -- Task frame expressed in the world frame
else
	tf = tcp_frame
	adaptive_task_frame = frame(vector(0,0,0.0))
end


-- ==================================== FORCE/TORQUE SIGNALS PRE-PROCESSING ===========================
function dead_zone(sign_0,dead_val)
    sign = conditional(abs(sign_0)-dead_val, sign_0 + conditional(sign_0, -dead_val, dead_val), constant(0))
    return sign
 end
 
Fx_after_dz  = dead_zone(Fx_raw,force_threshold)
Fy_after_dz  = dead_zone(Fy_raw,force_threshold)
Fz_after_dz  = dead_zone(Fz_raw,force_threshold)
Tx_after_dz  = dead_zone(Tx_raw,torque_threshold)
Ty_after_dz  = dead_zone(Ty_raw,torque_threshold)
Tz_after_dz  = dead_zone(Tz_raw,torque_threshold)

wr_after_dz = wrench(vector(Fx_after_dz,Fy_after_dz,Fz_after_dz),vector(Tx_after_dz,Ty_after_dz,Tz_after_dz))


if control_at_tcp then
	wr_tf   = ref_point(transform(rotation(inv(tcp_frame)*FT_sensor_frame),wr_after_dz) , -origin(inv(tcp_frame)*FT_sensor_frame))
else
	wr_tf   = ref_point(transform(rotation(inv(tf)*FT_sensor_frame),wr_after_dz) , -origin(inv(tf)*FT_sensor_frame))
end


Fx = coord_x(force(wr_tf))
Fy = coord_y(force(wr_tf))
Fz = coord_z(force(wr_tf))
Tx = coord_x(torque(wr_tf))
Ty = coord_y(torque(wr_tf))
Tz = coord_z(torque(wr_tf))



-- =============================== JOYSTICK WRENCH TRANSFORMATIONS ==============================

--Joystic input described by the right term of equation (5.16) of thesis 
joystick_input = wrench(vector(Vx_trans,Vy_trans,Vz_trans),vector(Vx_rot,Vy_rot,Vz_rot))

-- ============ The following steps are equivalent to transformation shown in equation (5.16) of thesis ===========
joystick_input_wrt_tcp = transform(inv(make_constant(rotation(tcp_frame))),joystick_input) -- joystick_input expressed in tcp frame coordinates
joystick_linear_wrt_tcp = force(joystick_input_wrt_tcp)
joystick_angular_wrt_tcp = torque(joystick_input_wrt_tcp)

beta = 10
moment_due_to_force = beta*cross(-1*origin(adaptive_task_frame), joystick_linear_wrt_tcp)
joystick_angular_at_tf = joystick_angular_wrt_tcp + moment_due_to_force

joystick_input_at_tf = wrench(joystick_linear_wrt_tcp,joystick_angular_at_tf)
wrench_joystick_tf =  transform(inv(rotation(adaptive_task_frame)),joystick_input_at_tf)
-- ============ The above steps are equivalent to transformation shown in equation (5.16) of thesis ===========

desired_force_x_at_tf = coord_x(force(wrench_joystick_tf))
desired_force_y_at_tf = coord_y(force(wrench_joystick_tf))
desired_force_z_at_tf = coord_z(force(wrench_joystick_tf))
desired_torque_x_at_tf = coord_x(torque(wrench_joystick_tf))
desired_torque_y_at_tf = coord_y(torque(wrench_joystick_tf))
desired_torque_z_at_tf = coord_z(torque(wrench_joystick_tf))

-- ============ The following steps are equivalent to applyingequation (5.19) of thesis ===========
desired_force_x_at_tf_assisted = alpha_sel*coord_x(force(wrench_joystick_tf)) --Assistance with selection matrix
desired_force_y_at_tf_assisted = alpha_sel*coord_y(force(wrench_joystick_tf)) --Assistance with selection matrix
desired_force_z_at_tf_assisted = alpha_prism*coord_z(force(wrench_joystick_tf)) --Assistance with selection matrix
desired_torque_x_at_tf_assisted = alpha_sel*coord_x(torque(wrench_joystick_tf)) --Assistance with selection matrix
desired_torque_y_at_tf_assisted = alpha_sel*coord_y(torque(wrench_joystick_tf)) --Assistance with selection matrix
desired_torque_z_at_tf_assisted = alpha_rev*coord_z(torque(wrench_joystick_tf)) --Assistance with selection matrix
-- ============ The above steps are equivalent to applyingequation (5.19) of thesis ===========


-- ============ The following steps correspond to computing equation (5.21), where variable twist_at_tcp corresponds to the term t_ND shown in the equation (expressed at the tcp) ============ 
twist_joystick = twist(vector(desired_force_x_at_tf,desired_force_y_at_tf,desired_force_z_at_tf),vector(desired_torque_x_at_tf,desired_torque_y_at_tf,desired_torque_z_at_tf))
twist_at_tf_wrt_tcp = transform(rotation(adaptive_task_frame),twist_joystick)
twist_at_tcp = ref_point(twist_at_tf_wrt_tcp,-origin(adaptive_task_frame))
-- ============ The above steps correspond to computing equation (5.21), where variable twist_at_tcp corresponds to the term t_ND shown in the equation (expressed at the tcp) ============ 


-- ============ The following steps correspond to computing equation (5.20), where the desired_force and desired_torque components correspond to the twist components of t_a in (5.20) expressed at the tcp ============
twist_joystick_assisted = twist(vector(desired_force_x_at_tf_assisted,desired_force_y_at_tf_assisted,desired_force_z_at_tf_assisted),vector(desired_torque_x_at_tf_assisted,desired_torque_y_at_tf_assisted,desired_torque_z_at_tf_assisted))
twist_at_tf_wrt_tcp_assisted = transform(rotation(adaptive_task_frame),twist_joystick_assisted)
twist_at_tcp_assisted = ref_point(twist_at_tf_wrt_tcp_assisted,-origin(adaptive_task_frame))

delta = constant(0.0001)

gamma_limit = 1/maximum(norm(transvel(twist_at_tcp))/max_v , maximum(norm(rotvel(twist_at_tcp))/max_omega, delta)) --Equation (5.22) of thesis
gamma_joy = maximum(norm(force(joystick_input)),norm(torque(joystick_input))) --Equation (5.23) of thesis
scaling_factor = gamma_limit*gamma_joy
-- scaling_factor = conditional((norm(transvel(twist_at_tcp))/max_v)-(norm(rotvel(twist_at_tcp))/max_omega), force_scale*max_v/norm(transvel(twist_at_tcp)), conditional((norm(rotvel(twist_at_tcp))/max_omega)-(norm(transvel(twist_at_tcp))/max_v), torque_scale*max_omega/norm(rotvel(twist_at_tcp)), 1.0))

desired_vel_x = scaling_factor*coord_x(transvel(twist_at_tcp_assisted))
desired_vel_y = scaling_factor*coord_y(transvel(twist_at_tcp_assisted))
desired_vel_z = scaling_factor*coord_z(transvel(twist_at_tcp_assisted))

desired_omega_x = scaling_factor*coord_x(rotvel(twist_at_tcp_assisted))
desired_omega_y = scaling_factor*coord_y(rotvel(twist_at_tcp_assisted))
desired_omega_z = scaling_factor*coord_z(rotvel(twist_at_tcp_assisted))
-- ============ The above steps correspond to computing equation (5.20), where variable twist_at_tcp corresponds to the term t_ND shown in the equation (expressed at the tcp) ============


-- =============================== INITIAL POSE ==============================

t_tf = origin(tf)
r_tf = rotation(tf)


-- =============================== ADMITTANCE CONSTRAINTS AT INSTANTANEOUS FRAME ==============================

if control_at_tcp then
	task_frame_i = inv(make_constant(tcp_frame))*tcp_frame
else
	task_frame_i = inv(make_constant(tf))*tf
end


pos_vec = origin(task_frame_i)
rot_vec = getRotVec(rotation(task_frame_i))

if translation_adm then

	Constraint{
		context=ctx,
		name="admittance_translation_x",
		model = coord_x(pos_vec),
		meas = -K_F*C_Fx*Fx,
		-- meas = -K_F*(C_trans_row_x_0*Fx + C_trans_row_x_1*Fy + C_trans_row_x_2*Fz),
		target = desired_vel_x, 
		K = 1,
		priority = 2,
		weight = 1,
		-- weight = abs(W_Fx),
	};

	Constraint{
		context=ctx,
		name="admittance_translation_y",
		model = coord_y(pos_vec),
		meas = -K_F*C_Fy*Fy,
		-- meas = -K_F*(C_trans_row_y_0*Fx + C_trans_row_y_1*Fy + C_trans_row_y_2*Fz),
		target = desired_vel_y,
		K = 1,
		priority = 2,
		weight = 1,
		-- weight = abs(W_Fy),
	};

	Constraint{
		context=ctx,
		name="admittance_translation_z",
		model = coord_z(pos_vec),
		meas = -K_F*C_Fz*Fz,
		-- meas = -K_F*(C_trans_row_z_0*Fx + C_trans_row_z_1*Fy + C_trans_row_z_2*Fz),
		target = desired_vel_z,
		K = 1,
		priority = 2,
		weight = 1,
		-- weight = abs(W_Fz),
	};

else
	tf_pos_init=initial_value(time,origin(tf))	
	Constraint {
		context         = ctx,
		name            = "keep_trans",
		expr            = origin(tf)-tf_pos_init,
		weight          = 1000,
		priority        = 2,
		K               = 4
	};


end


if rotation_adm then


	Constraint{
		context=ctx,
		name="admittance_rotation_x",
		model = coord_x(rot_vec),
		meas = -K_T*C_Tx*Tx,
		-- meas = -K_T*(C_rot_row_x_0*Tx + C_rot_row_x_1*Ty + C_rot_row_x_2*Tz + C_mix_row_x_0*Fx + C_mix_row_x_1*Fy + C_mix_row_x_2*Fz),
		target = desired_omega_x,
		K = 1,
		priority = 2,
		weight = 1,
	};

	Constraint{
		context=ctx,
		name="admittance_rotation_y",
		model = coord_y(rot_vec),
		meas = -K_T*C_Ty*Ty,
		-- meas = -K_T*(C_rot_row_y_0*Tx + C_rot_row_y_1*Ty + C_rot_row_y_2*Tz + C_mix_row_y_0*Fx + C_mix_row_y_1*Fy + C_mix_row_y_2*Fz),
		target = desired_omega_y,
		K = 1,
		priority = 2,
		weight = 1,
	};

	Constraint{
		context=ctx,
		name="admittance_rotation_z",
		model = coord_z(rot_vec),
		meas = -K_T*C_Tz*Tz,
		-- meas = -K_T*(C_rot_row_z_0*Tx + C_rot_row_z_1*Ty + (C_rot_row_z_2)*Tz + C_mix_row_z_0*Fx + C_mix_row_z_1*Fy + C_mix_row_z_2*Fz),
		target = desired_omega_z,
		K = 1,
		priority = 2,
		weight = 1,
	};

else
	r=initial_value(time,r_tf)		-- initial value of the translation of the end efector

	Constraint {
		context         = ctx,
		name            = "keep_rot",
		expr            = r_tf*inv(r),
		weight          = 1,
		priority        = 2,
		K               = 4
	};
end


-- The following are workspace limits applied in different frames of the robot, specifically for the KUKA iiwa setup
-- Constraint{
-- 	context = ctx,
-- 	name    = "workspace_limit_TCP_x",
-- 	expr    = coord_x(origin(tcp_frame)),
-- 	target_lower = 0.35,
-- 	K       = 4,
-- 	weight  = 10000,
-- 	priority= 2
-- };

-- safety_frame_wrist = FT_sensor_frame*frame(vector(0,0,0.25))
-- Constraint{
-- 	context = ctx,
-- 	name    = "workspace_limit_FT_frame_z",
-- 	expr    = coord_z(origin(safety_frame_wrist)),
-- 	-- expr    = coord_z(origin(FT_sensor_frame)),
-- 	target_lower = 0.17,
-- 	K       = 4,
-- 	weight  = 10000,
-- 	priority= 2
-- };
-- ============================== OUTPUT THROUGH PORTS===================================

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tcp",coord_x(origin(tf)))
ctx:setOutputExpression("y_tcp",coord_y(origin(tf)))
ctx:setOutputExpression("z_tcp",coord_z(origin(tf)))
--

