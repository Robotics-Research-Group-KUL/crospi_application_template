--  Copyright (c) 2025 KU Leuven, Belgium
--
--  GNU Lesser General Public License Usage
--  Alternatively, this file may be used under the terms of the GNU Lesser
--  General Public License version 3 as published by the Free Software
--  Foundation and appearing in the file LICENSE.LGPLv3 included in the
--  packaging of this file. Please review the following information to
--  ensure the GNU Lesser General Public License version 3 requirements
--  will be met: https://www.gnu.org/licenses/lgpl.html.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Lesser General Public License for more details.

require("context")
require("geometric")
require("math")

reqs = require("task_requirements")

task_description = [[pushing]]

-- ========================================= PARAMETERS ===================================
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="max_door_angle", description="Angle in degrees that the door should be opened (limited by geometry)", 
        default = 65, required=true,maximum=65}),    
    reqs.params.scalar({name="max_time", description="Maximum duration [s] of the task: it finishes when this time has elapsed. 0 disables this monitor", 
                        default = 0, required=false, minimum = 0.0}),   
    reqs.params.scalar({name="desired_force", description="Desired force to pull", default = 10, required=true}),    
    reqs.params.array({name="stiffness_F", type=reqs.array_types.number, default={4000, 4000, 2400},
                            description="Diagonal elements [N/m] of the translational stiffness, as [x,y,z] along the axes of the control frame, used as model of the force control: the velocity along each axis is K_force*(desired force - measured force)/K_F. Lower values make the robot faster and more compliant along that axis, higher values slower and stiffer. The compliant axes should have a low value so that the robot follows the object (e.g. the arc of a door) with a small force", required=true, minimum = 0.0, minItems = 3, maxItems = 3}),
    reqs.params.array({name="stiffness_T", type=reqs.array_types.number, default={100, 100, 100},
                            description="Diagonal elements [Nm/rad] of the rotational stiffness, as [x,y,z] around the axes of the control frame, used as model of the torque control: the angular velocity around each axis is K_torque*(0 - measured torque)/K_T, i.e. the robot rotates with the object (e.g. a door handle that turns) with a small torque. Lower values make the robot rotate faster for the same torque", required=true, minimum = 0.0, minItems = 3, maxItems = 3}),
    reqs.params.scalar({name="force_gain", description="Feedback gain [1/s] of the force control constraints along x, y and z (inverse of their time constant)", 
                        default = 5, required=true, minimum = 0.0}),    
    reqs.params.scalar({name="torque_gain", description="Feedback gain [1/s] of the torque control constraints around x, y and z (inverse of their time constant)", 
                        default = 4, required=true, minimum = 0.0}),
    reqs.params.string({name="task_frame", description="Name of the robot frame (from the robot model) that is controlled, e.g. the tool center point.", 
                        default = "tcp_frame", required=true}),
    reqs.params.string({name="FT_sensor_frame", description="Name of the robot frame (from the robot model) of the force/torque sensor. The measured wrench of the input channel is expressed in this frame, with its origin as reference point", 
                        default = "FT_sensor_frame", required=true}),
    reqs.params.string({name="wrench_input_channel", description="Name of the input channel (type wrench) from which the measured wrench of the force/torque sensor is read: the channel fed directly by the robot driver/simulator (name_expr_cartesian_wrench of the MuJoCo simulator) or the varname of an input handler that provides it. The wrench is the wrench exerted by the end effector on the environment", 
                        default = "wrench_input", required=false}),
})

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"),
    param.get("FT_sensor_frame"),
})
robot_joints    = robot.robot_joints
task_frame      = robot.getFrame(param.get("task_frame"))
FT_sensor_frame = robot.getFrame(param.get("FT_sensor_frame"))


-- ========================================= PARAMETERS ===================================
max_door_angle   = param.get("max_door_angle")
max_time         = param.get("max_time")
desired_force    = param.get("desired_force")
stiffness_F      = param.get("stiffness_F")
stiffness_T      = param.get("stiffness_T")
force_gain       = param.get("force_gain")
torque_gain      = param.get("torque_gain")
sensed_wrench    = ctx:createInputChannelWrench(param.get("wrench_input_channel"))

-- ========================================= Variables coming from topic input handlers ===================================


function transform_wrench( F, w)
    local F = cached(F)
    local R = rotation(F)
    local p = origin(F)
    local F_result = cached( R*force(w))
    local T_result = R* torque(w) + cross(p,F_result)
    return wrench(F_result,T_result)
end



wrench_tf = transform_wrench( inv(task_frame)* FT_sensor_frame, sensed_wrench)    -- transformed from sensor frame to task frame
Fx = coord_x(force(wrench_tf))
Fy = coord_y(force(wrench_tf))
Fz = coord_z(force(wrench_tf))
Tx = coord_x(torque(wrench_tf))
Ty = coord_y(torque(wrench_tf))
Tz = coord_z(torque(wrench_tf))


-- =============================== INSTANTANEOUS FRAME ==============================
-- pose of the control frame w.r.t. its pose at the current instant (zero, but its derivative is the velocity of the
-- control frame expressed in the control frame)
tf_inst  = inv(make_constant(task_frame))*task_frame
pos_inst = origin(tf_inst)                -- 3-vector with a translation
rot_inst = getRotVec(rotation(tf_inst))   -- 3-vector with a rotational displacement





-- =============================== FORCE AND TORQUE CONTROL ==============================

-- ================================ start of TODO block ======================================
-- stiffness_F and stiffness_T : 1-based arrays in lua, i.e. stiffness_F[1] is the first element.
-- Directions i = 1..3 translation and 4..6 rotation 
-- stiffness translational  stiffness_F[i] (already conform to the setup)
-- stiffness torque         stiffness_T[i] (already conform to the setup)
-- priority=1 is hard constraint,  priority=2 is soft constraint
-- typical gain: K=5 (expressed in [s^-1] )  (for both force and velocity constraints)
--
-- Force constraint:
--    model    = constant(stiffness_F[i]) * coord_x(pos_inst),   
--    meas     = Fx   or  Fy, Fz    (component of wrench_tf, see above)
--    target   = .... -- expressed in [N]
-- Torque constraint:
--    model    = constant(stiffness_T[i]) * coord_x(rot_inst),   
--    meas     = Tx   or Ty, Tz    (component of wrench_tf, see above)
--    target   = ....  -- expressed in [Nm]
--
-- Velocity constraint:
--   expr     = coord_x[pos_inst]
--   target   = desired_velocity* time
--   K        = 0                  (why zero?)
--  
--   if desired_velocity is an expression, use make_constant(desired_velocity),
--   otherwise the derivative of target is not desired_velocity
--
-- Rotatinal Velocity constraint:
--   expr     = coord_x[rot_inst]
--   target   = desired_rotvelocity* time
--   K        = 0                  (why zero?)
-- 
--   if desired_rotvelocity is an expression, use make_constant(desired_rotvelocity),
--   otherwise the derivative of target is not desired_rotvelocity


-- for each direction you want to control:
Constraint{
    context  = ctx,
    name     = "my_unique name",
    model    = ...,
    meas     = ...,
    target   = ...,
    K        = 5,
    priority = ... 
}

-- ...

-- ...


-- ================================ end of TODO block ======================================


initial_taskframe_z = initial_value(time, unit_z(rotation(task_frame)))
current_taskframe_z = unit_z(rotation(task_frame))

-- limit expression x to [lo, hi] (lo, hi: numbers or expressions)
function clamp(x, lo, hi)
    return conditional(x - lo, conditional(hi - x, x, hi), lo)
end

door_angle = acos( dot(initial_taskframe_z, current_taskframe_z))/pi*180.0

-- =============================== MONITORS ==============================
Monitor{
    context = ctx, 
    name       = "angle_achieved",
    upper      = max_door_angle,
    actionname = "exit", 
    expr       = door_angle
}

if max_time > 0 then
    Monitor{
        context    = ctx,
        name       = "finish_after_max_time",
        upper      = max_time,
        actionname = "exit",
        expr       = time,
    }
end

-- =============================== OUTPUT ==============================
roll_tf, pitch_tf, yaw_tf = getRPY(rotation(task_frame))

--ctx:setOutputExpression("travel"       , travel)
ctx:setOutputExpression("x_tf"         , coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf"         , coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf"         , coord_z(origin(task_frame)))
ctx:setOutputExpression("roll_tf"      , roll_tf)
ctx:setOutputExpression("pitch_tf"     , pitch_tf)
ctx:setOutputExpression("yaw_tf"       , yaw_tf)
ctx:setOutputExpression("x_force"      , Fx)
ctx:setOutputExpression("y_force"      , Fy)
ctx:setOutputExpression("z_force"      , Fz)
ctx:setOutputExpression("x_torque"     , Tx)
ctx:setOutputExpression("y_torque"     , Ty)
ctx:setOutputExpression("z_torque"     , Tz)
ctx:setOutputExpression("task_frame_c" , task_frame)
ctx:setOutputExpression("door_angle" , door_angle)
