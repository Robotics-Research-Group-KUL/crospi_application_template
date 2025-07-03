require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")

reqs = require("task_requirements")
 
task_description = "This task specification allows keep the initial pose of one frame with respect to the other one steady."

-- ========================================= PARAMETERS ===================================
 
param = reqs.parameters(task_description,{
    reqs.params.string({name="task_frame", description="Name of frame that will be kept still with respect to a base frame which will continuously adapt", default = "tcp_frame", required=false}),
    reqs.params.string({name="base_frame", description="The specified task frame will be kept still relative to this base frame that will continuously adapt. This frame should come from a PoseInputHandler or a TFInputHandler", default = "base_frame", required=false}),
    reqs.params.scalar({name="execution_time", description="(optional) time (seconds) that the task should run before stopping. The task will only automatically stop if execution_time>0", default = 0, required=false, minimum = -1}),
    reqs.params.bool({name="activate_linear", description="If true, the linear velocities of the joystick will control the linear velocities of the defined frame", default = true, required=true}),
    reqs.params.bool({name="activate_angular", description="If true, the angular velocities of the joystick will control the angular velocities of the defined frame", default = false, required=true}),

})

-- TODO: Change order of quaterinions in the skill.
-- TODO: Check what happens when quaterion is not valid.
-- TODO: Change tcp_2_tf.

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
})
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))

base_frame   = ctx:createInputChannelFrame(param.get("base_frame"))

T_tcp_bf = inv(base_frame)*task_frame --Transformation from TCP to base_frame 

T_tcp_bf_target = initial_value(time, T_tcp_bf) --Initial transformation from TCP to base_frame 


-- ========================== CONSTRAINT SPECIFICATION =================================

if param.get("activate_linear") then

    Constraint{
        context = ctx,
        name    = "maintain_relative_posisition",
        expr    = origin(T_tcp_bf) - origin(T_tcp_bf_target),
        K       = 2,
        weight  = 1,
        priority= 2
    }
else
    Constraint{
        context = ctx,
        name    = "keep_translation_constant",
        expr    = origin(task_frame),
        target  = initial_value(time, origin(task_frame)),
        K       = 4,
        weight  = 1,
        priority= 2
    };
end



if param.get("activate_angular") then
    
    Constraint{
        context = ctx,
        name    = "maintain_relative_rotation",
        expr    = inv(rotation(T_tcp_bf_target))*rotation(T_tcp_bf),
        K       = 1,
        weight  = 1,
        priority= 2
    }
else
    Constraint{
        context = ctx,
        name    = "keep_rotation_constant",
        expr    = rotation(task_frame)*initial_value(time, rotation(task_frame)),
        K       = 4,
        weight  = 1,
        priority= 2
    };

end


-- =========================== MONITOR ============================================
-- Monitor{
--         context=ctx,
--         name='finish_after_motion',
--         upper=0.0,
--         actionname='exit',
--         expr=time-get_duration(mp) - constant(1)
-- }

-- Monitor{
--     context=ctx,
--     name='finish_and_trigger_console',
--     upper=0.0,
--     actionname='debug',
--     expr=time- constant(1)
-- }


-- Monitor{
--     context=ctx,
--     name='portevent_test',
--     upper=0.0,
--     actionname='portevent',
--     argument = "test_event",
--     expr=time-0.5
-- }

-- Monitor{
--     context=ctx,
--     name='event_test',
--     upper=0.0,
--     actionname='event',
--     argument = "test_event",
--     expr=time-0.5
-- }

execution_time = param.get("execution_time")

if(execution_time>0) then 
    Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-constant(execution_time)
    }
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