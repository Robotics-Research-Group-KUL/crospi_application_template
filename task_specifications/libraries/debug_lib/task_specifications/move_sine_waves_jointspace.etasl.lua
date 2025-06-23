require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
reqs = require("task_requirements")

task_description = "Generates independent sine waves for each joint with given amplitudes and frequencies."

param = reqs.parameters(task_description,{
    reqs.params.array({name="amplitudes", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, description="Array with the amplitudes of the sine waves for each joint in [rad]. Its values correspond to the defined robot.robot_joints in the same order", required=true, minimum = -6.28318, maximum=6.28318, minItems = 1}),
    reqs.params.array({name="frequencies", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, description="Array with the frequencies of the sine waves for each joint in [Hz]. Its values correspond to the defined robot.robot_joints in the same order", required=true, minimum = -6.28318, maximum=6.28318, minItems = 1}),
    reqs.params.scalar({name="execution_time", description="Time (seconds) that the task should run before stopping ", default = 5, required=false, minimum = 0}),
})

robot = reqs.robot_model({"tcp_frame"})


-- ========================================= PARAMETERS ===================================

amplitudes = param.get("amplitudes")

if #robot.robot_joints ~= #amplitudes then
    error("The number of robot.robot_joints and the specified number of amplitudes must coincide")
end


frequencies = param.get("frequencies")

if #robot.robot_joints ~= #frequencies then
    error("The number of robot.robot_joints and the specified number of frequencies must coincide")
end

execution_time = param.get("execution_time")



-- ========================= CONSTRAINT SPECIFICATION ========================

tgt         = {} -- target value
for i=1,#robot.robot_joints do
    local current_jnt   = ctx:getScalarExpr(robot.robot_joints[i])

    local tgt = amplitudes[i]*sin(2*math.pi*frequencies[i]*time) + initial_value(time, current_jnt)

    Constraint{
        context=ctx,
        name="joint_trajectory"..i,
        expr= current_jnt - tgt ,
        priority = 2,
        K=2
    };

    
end


-- =================================== MONITOR TO FINISH THE MOTION ========================

Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-constant(execution_time)
}

-- Monitor{
--     context=ctx,
--     name='finish_after_on_ended',
--     upper=0.0,
--     actionname='debug',
--     expr=time-constant(1)
-- }


tcp_frame = robot.getFrame("tcp_frame")

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tcp",coord_x(origin(tcp_frame)))
ctx:setOutputExpression("y_tcp",coord_y(origin(tcp_frame)))
ctx:setOutputExpression("z_tcp",coord_z(origin(tcp_frame)))
