--  Copyright (c) 2025 KU Leuven, Belgium
--
--  Author: Santiago Iregui
--  email: <santiago.iregui@kuleuven.be>
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
-- worldmodel=require("worldmodel")
require("math")
reqs = require("task_requirements")


-- ========================================= PARAMETERS ===================================
task_description = [[
Teleoperates a task frame of the robot (e.g. the end effector) in Cartesian velocity with a 6D joystick (a.k.a. spacemouse).
Input channel: named by joystick_input_channel (default "joystick_input"), of type Twist, typically provided by a TwistInputHandler subscribed to the joystick topic. When activate_linear is true, the linear velocity of the task frame along the X,Y,Z axes of the robot base frame (root frame of the robot model) follows the linear part of the twist times linear_scale; otherwise the position of the task frame is kept fixed at its initial value. When activate_angular is true, the time derivatives of the rotation-vector components of the task frame orientation (w.r.t. the robot base frame) follow the angular part of the twist times angular_scale; otherwise the orientation is kept fixed at its initial value.
The task runs until stopped externally, unless execution_time > 0, in which case it exits after execution_time seconds. Outputs: time, position (x_tf, y_tf, z_tf) [m] and orientation quaternion (qx_tf, qy_tf, qz_tf, qw_tf) of the task frame w.r.t. the robot base frame.]]

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="linear_scale", description="Scale factor applied to the linear part of the joystick twist: commanded linear velocity of the task frame [m/s] = joystick linear input * linear_scale. Larger values make the robot move faster for the same joystick deflection", default = 0.3, required=false}),
    reqs.params.scalar({name="angular_scale", description="Scale factor applied to the angular part of the joystick twist: commanded angular rate of the task frame [rad/s] = joystick angular input * angular_scale. Larger values make the robot rotate faster for the same joystick deflection", default = 0.3, required=false}),
    reqs.params.string({name="task_frame", description="Name of the robot frame (as defined in the robot model, e.g. URDF link or tcp_frame) that is teleoperated in Cartesian space", default = "tcp_frame", required=false}),
    reqs.params.bool({name="activate_linear", description="If true, the linear part of the joystick twist controls the linear velocity of the task frame (along the robot base frame axes). If false, the position of the task frame is kept fixed at its initial value", default = true, required=true}),
    reqs.params.bool({name="activate_angular", description="If true, the angular part of the joystick twist controls the rotation of the task frame. If false, the orientation of the task frame is kept fixed at its initial value", default = false, required=true}),
    reqs.params.scalar({name="execution_time", description="Time [s] after which the task stops automatically (exit monitor). If zero (default), the task does not stop by itself and runs until stopped externally", default = 0, required=false, minimum = 0}),
    reqs.params.scalar({name="K_fixed_position", description="Feedback gain [1/s] of the constraint that keeps the position of the task frame fixed at its initial value (used when activate_linear is false). Larger values correct deviations faster", default = 4, required=false, minimum = 0}),
    reqs.params.scalar({name="K_fixed_rotation", description="Feedback gain [1/s] of the constraint that keeps the orientation of the task frame fixed at its initial value (used when activate_angular is false). Larger values correct deviations faster", default = 4, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_linear", description="Weight (dimensionless) of the translational constraints (the three linear velocity constraints, or the fixed-position constraint when activate_linear is false) relative to the other constraints of the same priority. Only relative weights matter: a higher weight makes them dominate when constraints conflict", default = 1, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_angular", description="Weight (dimensionless) of the rotational constraints (the three angular velocity constraints, or the fixed-orientation constraint when activate_angular is false) relative to the other constraints of the same priority. Only relative weights matter: a higher weight makes them dominate when constraints conflict", default = 1, required=false, minimum = 0}),

    reqs.params.string({name="joystick_input_channel", description="Name of the input channel (type twist) from which the joystick velocities are read: the varname of the input handler that provides it (e.g. a TwistInputHandler subscribed to /spacenav/twist). Change it to use another joystick or input handler", default = "joystick_input", required=false}),
})
linear_scale    = constant(param.get("linear_scale"))
angular_scale    = constant(param.get("angular_scale"))
execution_time = param.get("execution_time")


-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({ --This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    --Add all frames that are required by the task specification
}) 
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))

-- ========================================= Variables coming from topic input handlers ===================================
joystick_input   = ctx:createInputChannelTwist(param.get("joystick_input_channel"))

-- joystick_input = twist(vector(0,0,-0.05),vector(0,0,0))


-- =============================== INSTANTANEOUS FRAME ==============================

tf_inst = task_frame
-- tf_inst = inv(make_constant(task_frame))*task_frame


if param.get("activate_linear") then

    desired_vel_x = coord_x(transvel(joystick_input))*linear_scale
    desired_vel_y = coord_y(transvel(joystick_input))*linear_scale
    desired_vel_z = coord_z(transvel(joystick_input))*linear_scale
    -- Translation velocities
    Constraint{
        context = ctx,
        name    = "x_velocity",
        expr    = coord_x(origin(tf_inst)),
        target  = desired_vel_x*time,
        K       = 0,
        weight  = param.get("weight_linear"),
        priority= 2
    };
    
    Constraint{
        context = ctx,
        name    = "y_velocity",
        expr    = coord_y(origin(tf_inst)),
        target  = desired_vel_y*time,
        K       = 0,
        weight  = param.get("weight_linear"),
        priority= 2
    };
    
    Constraint{
        context = ctx,
        name    = "z_velocity",
        expr    = coord_z(origin(tf_inst)),
        target  = desired_vel_z*time,
        K       = 0,
        weight  = param.get("weight_linear"),
        priority= 2
    };
else
    Constraint{
        context = ctx,
        name    = "keep_translation_constant",
        expr    = origin(tf_inst),
        target  = initial_value(time, origin(tf_inst)),
        K       = param.get("K_fixed_position"),
        weight  = param.get("weight_linear"),
        priority= 2
    };

end

if param.get("activate_angular") then

    desired_omega_x = coord_x(rotvel(joystick_input))*angular_scale
    desired_omega_y = coord_y(rotvel(joystick_input))*angular_scale
    desired_omega_z = coord_z(rotvel(joystick_input))*angular_scale

    -- Orientation velocities
    Constraint{
        context = ctx,
        name    = "x_angular",
        expr    = coord_x(getRotVec(rotation(tf_inst))) - desired_omega_x*time,
        K       = 0,
        weight  = param.get("weight_angular"),
        priority= 2
    };

    Constraint{
        context = ctx,
        name    = "y_angular",
        expr    = coord_y(getRotVec(rotation(tf_inst))) - desired_omega_y*time,
        K       = 0,
        weight  = param.get("weight_angular"),
        priority= 2
    };

    Constraint{
        context = ctx,
        name    = "z_angular",
        expr    = coord_z(getRotVec(rotation(tf_inst))) - desired_omega_z*time,
        K       = 0,
        weight  = param.get("weight_angular"),
        priority= 2
    };

else
    Constraint{
        context = ctx,
        name    = "keep_rotation_constant",
        expr    = rotation(tf_inst)*initial_value(time, rotation(tf_inst)),
        K       = param.get("K_fixed_rotation"),
        weight  = param.get("weight_angular"),
        priority= 2
    };


end


shape = "cylinder"
-- shape = "cone"

moving_fixture = false --Change to true to make the fixture move in a sine wave

origin_vf_x = initial_value(time, coord_x(origin(task_frame))) --defines origin of the virtual fixture in the initial X position of the task frame
origin_vf_y = initial_value(time, coord_y(origin(task_frame))) --defines origin of the virtual fixture in the initial Y position of the task frame
origin_vf_z = 0.20 --height of the bottom side of the virtual fixture (table height = 0.15)

if moving_fixture then
    freq = 0.2
    amplitude = 0.15
    origin_vf_x = initial_value(time, coord_x(origin(task_frame))) + amplitude*sin(2*3.14*freq*time)

end

if shape == "cylinder" then
    rad1 = 0.05
    length = 0.4

    rad2 = rad1 --for cylinder there is only one radius (but we use a cone as a marker, so we need both)
    radius = rad1 -- for cylinder there is only one radius

elseif shape == "cone" then
    rad1 = 0.03 --Bottom radius of the cone (this one is at position vector(origin_vf_x,origin_vf_y,origin_vf_z))
    rad2 = 0.1 -- Upper radius of the cone (this one is at position vector(origin_vf_x,origin_vf_y,origin_vf_z + length))
    length = 0.3    

    -- A cone can be modeled a cylinder with the radius being a linear function of the height:
    slope = (rad2-rad1)/length
    y_intercept = rad1 - slope*origin_vf_z
    radius = slope*coord_z(origin(task_frame)) +y_intercept 
end

-- Equation of a cylinder in X-Y plane (expr): (X_tf - X_cone)^2 + (X_tf - X_cone)^2 = radius^2
-- By making it an inequality, we ensure the task_frame is inside the cylinder

circle_equation =  (coord_x(origin(task_frame))-origin_vf_x)*(coord_x(origin(task_frame))-origin_vf_x)+(coord_y(origin(task_frame))-origin_vf_y)*(coord_y(origin(task_frame))-origin_vf_y)

Constraint{
    context = ctx,
    name    = "virtual_fixture_shape",
    expr    = circle_equation,
    target_upper = radius*radius,
    K       = 4,
    weight  = 200,
    priority= 2 -- soft virtual fixture
    -- priority= 1 --hard virtual fixture
};

Constraint{
    context = ctx,
    name    = "virtual_fixture_top_and_bottom",
    expr    = coord_z(origin(task_frame)),
    target_lower = origin_vf_z,
    target_upper = origin_vf_z + length,
    K       = 4,
    weight  = 200,
    priority= 2 -- soft virtual fixture
    -- priority= 1 --hard virtual fixture
};



quat_tf = toQuat(rotation(task_frame))

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
ctx:setOutputExpression("qx_tf",coord_x(vec(quat_tf)))
ctx:setOutputExpression("qy_tf",coord_y(vec(quat_tf)))
ctx:setOutputExpression("qz_tf",coord_z(vec(quat_tf)))
ctx:setOutputExpression("qw_tf",scalar(quat_tf))


if (execution_time > 0) then
    Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-constant(execution_time)
    }
end

-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(task_frame))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)
