require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
reqs = require("task_requirements")

task_description = "Runs robot in idle state, e.g. so that joints_states are published."

param = reqs.parameters(task_description,{

})


robot = reqs.robot_model({
    "tcp_frame",
    -- "forearm",
    --Add all frames that are required by the task specification
}) 

task_frame = robot.getFrame("tcp_frame")


ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))

roll_tf,pitch_tf,yaw_tf = getRPY(rotation(task_frame))
ctx:setOutputExpression("roll_tf",roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf",yaw_tf)




