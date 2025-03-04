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
})
linear_scale    = constant(param.get("linear_scale"))
angular_scale    = constant(param.get("angular_scale"))


-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({ --This function loads the robot model and checks that all required frames are available
    "tcp_frame",
    --Add all frames that are required by the task specification
}) 
robot_joints = robot.robot_joints
task_frame = robot.getFrame("tcp_frame")

-- ========================================= Variables coming from topic input handlers ===================================
joystick_input   = ctx:createInputChannelTwist("joystick_input")
-- joystick_input = twist(vector(0,0,-0.05),vector(0,0,0))


-- =============================== INSTANTANEOUS FRAME ==============================

-- tf_inst = inv(make_constant(task_frame))*task_frame

desired_vel_x = coord_x(transvel(joystick_input))*linear_scale
desired_vel_y = coord_y(transvel(joystick_input))*linear_scale
desired_vel_z = coord_z(transvel(joystick_input))*linear_scale

desired_omega_x = coord_x(rotvel(joystick_input))*angular_scale
desired_omega_y = coord_y(rotvel(joystick_input))*angular_scale
desired_omega_z = coord_z(rotvel(joystick_input))*angular_scale

-- Translation velocities
Constraint{
    context = ctx,
    name    = "x_velocity",
    expr    = coord_x(origin(task_frame)),
    target  = desired_vel_x*time,
    K       = 0,
    weight  = 1,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "y_velocity",
    expr    = coord_y(origin(task_frame)),
    target  = desired_vel_y*time,
    K       = 0,
    weight  = 1,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "z_velocity",
    expr    = coord_z(origin(task_frame)),
	target  = desired_vel_z*time,
    K       = 0,
    weight  = 1,
    priority= 2
};

-- Orientation velocities
Constraint{
    context = ctx,
    name    = "x_angular",
    expr    = coord_x(getRotVec(rotation(task_frame))) - desired_omega_x*time,
    K       = 0,
    weight  = 1,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "y_angular",
    expr    = coord_y(getRotVec(rotation(task_frame))) - desired_omega_y*time,
    K       = 0,
    weight  = 1,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "z_angular",
    expr    = coord_z(getRotVec(rotation(task_frame))) - desired_omega_z*time,
    K       = 0,
    weight  = 1,
    priority= 2
};



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
