require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")


-- ======================================== FRAMES ========================================

tf = ee

last_joint   = ctx:getScalarExpr(robot_joints[6])
-- last_joint   = ctx:getScalarExpr(robot_joints[7])

Constraint{
    context=ctx,
    name="joint_trajectory",
    expr= last_joint - sin(time) * 10*3.1416/180,
    priority = 2,
    K=4
};


ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tcp",coord_x(origin(tf)))
ctx:setOutputExpression("y_tcp",coord_y(origin(tf)))
ctx:setOutputExpression("z_tcp",coord_z(origin(tf)))
ctx:setOutputExpression("tf",tf)




-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(tf)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(tf)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(tf)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(tf))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)
