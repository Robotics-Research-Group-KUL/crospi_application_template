require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
reqs = require("task_requirements")


-- ========================================= PARAMETERS ===================================
task_description = "This task specification allows to control the angular and linear velocity the end effector via a 6D joystick (a.k.a. spacemouse)."

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="linear_scale", description="Scales the magnitude of the linear velocity coming from the joystick", default = 1, required=false}),
    reqs.params.scalar({name="angular_scale", description="Scales the magnitude of the angular velocity coming from the joystick", default = 1, required=false}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=false}),
    reqs.params.bool({name="activate_linear", description="If true, the linear velocities of the joystick will control the linear velocities of the defined frame", default = true, required=true}),
    reqs.params.bool({name="activate_angular", description="If true, the angular velocities of the joystick will control the angular velocities of the defined frame", default = false, required=true}),

})
linear_scale    = constant(param.get("linear_scale"))
angular_scale    = constant(param.get("angular_scale"))

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({ --This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    --Add all frames that are required by the task specification
}) 
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))

-- ========================================= Variables coming from topic input handlers ===================================
joystick_input   = ctx:createInputChannelTwist("joystick_input")


joint_vel_0   = ctx:createInputChannelScalar("joint_vel_0")
joint_vel_1   = ctx:createInputChannelScalar("joint_vel_1")
joint_vel_2   = ctx:createInputChannelScalar("joint_vel_2")
joint_vel_3   = ctx:createInputChannelScalar("joint_vel_3")

joint_torque_0   = ctx:createInputChannelScalar("joint_torque_0")
joint_torque_1   = ctx:createInputChannelScalar("joint_torque_1")
joint_torque_2   = ctx:createInputChannelScalar("joint_torque_2")
joint_torque_3   = ctx:createInputChannelScalar("joint_torque_3")

joint_current_0   = ctx:createInputChannelScalar("joint_current_0")
joint_current_1   = ctx:createInputChannelScalar("joint_current_1")
joint_current_2   = ctx:createInputChannelScalar("joint_current_2")
joint_current_3   = ctx:createInputChannelScalar("joint_current_3")


task_frame_pos   = ctx:createInputChannelVector("task_frame_pos")
task_frame_quat   = ctx:createInputChannelRotation("task_frame_quat")
task_frame_twist   = ctx:createInputChannelTwist("task_frame_twist")
task_frame_wrench   = ctx:createInputChannelWrench("task_frame_wrench")
base_pos   = ctx:createInputChannelVector("base_pos")
base_quat   = ctx:createInputChannelRotation("base_quat")
base_twist   = ctx:createInputChannelTwist("base_twist")
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
        weight  = 1,
        priority= 2
    };
    
    Constraint{
        context = ctx,
        name    = "y_velocity",
        expr    = coord_y(origin(tf_inst)),
        target  = desired_vel_y*time,
        K       = 0,
        weight  = 1,
        priority= 2
    };
    
    Constraint{
        context = ctx,
        name    = "z_velocity",
        expr    = coord_z(origin(tf_inst)),
        target  = desired_vel_z*time,
        K       = 0,
        weight  = 1,
        priority= 2
    };
else
    Constraint{
        context = ctx,
        name    = "keep_translation_constant",
        expr    = origin(tf_inst),
        target  = initial_value(time, origin(tf_inst)),
        K       = 4,
        weight  = 1,
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
        weight  = 1,
        priority= 2
    };

    Constraint{
        context = ctx,
        name    = "y_angular",
        expr    = coord_y(getRotVec(rotation(tf_inst))) - desired_omega_y*time,
        K       = 0,
        weight  = 1,
        priority= 2
    };

    Constraint{
        context = ctx,
        name    = "z_angular",
        expr    = coord_z(getRotVec(rotation(tf_inst))) - desired_omega_z*time,
        K       = 0,
        weight  = 1,
        priority= 2
    };

else
    Constraint{
        context = ctx,
        name    = "keep_rotation_constant",
        expr    = rotation(tf_inst)*initial_value(time, rotation(tf_inst)),
        K       = 4,
        weight  = 1,
        priority= 2
    };


end

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
