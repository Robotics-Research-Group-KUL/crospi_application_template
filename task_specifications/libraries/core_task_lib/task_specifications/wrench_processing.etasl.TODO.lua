require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
require("etasl_parameters")

sensed_wrench   = ctx:createInputChannelWrench("wrench_input")

function dead_zone(signal,dead_val)
    signal_dead_zone = conditional(abs(signal)-dead_val, signal + conditional(signal, -dead_val, dead_val), constant(0))
    return signal_dead_zone
end

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