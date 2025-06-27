require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
reqs = require("task_requirements")


-- ========================================= PARAMETERS ===================================
task_description = "This task specification allows to control the angular and linear velocity the end effector via a 6D joystick (a.k.a. spacemouse)."

param = reqs.parameters(task_description,{
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=true}),
    reqs.params.string({name="vector_name_var", description="Name of the vector where the tfinputhandler publishes the vector", default = "vector_var", required=true}),

})


-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({ --This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
--Add all frames that are required by the task specification
}) 
robot_joints = robot.robot_joints
task_frame = robot.getFrame("tcp_frame")

-- ========================================= Variables coming from topic input handlers ===================================
vector_var   = ctx:createInputChannelVector(param.get("vector_name_var"))

target_rot = initial_value(time, rotation(task_frame))

target_frame = frame(target_rot, vector_var)
-- joystick_input = twist(vector(0,0,-0.05),vector(0,0,0))


-- =============================== INSTANTANEOUS FRAME ==============================

-- tf_inst = inv(make_constant(task_frame))*task_frame

Constraint{
    context = ctx,
    name    = "follow_frame",
    expr    = inv(target_frame)*task_frame,
    K       = 3,
    weight  = 1,
    priority= 2
}

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tcp",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tcp",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tcp",coord_z(origin(task_frame)))
ctx:setOutputExpression("tf",task_frame)




-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(task_frame))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)
