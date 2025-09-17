require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
reqs = require("task_requirements")

task_description = "Moves a subset of joints (in joint space) to a target pose specified using joint angles."

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity rad/s", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration rad/s^2", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.array({name="target_joints", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, description="Array with target angles. Its values correspond to the defined in the parameter robot_joints (in the same order)", required=true, minimum = -360, maximum=360, minItems = 1}),
    reqs.params.array({name="robot_joints", type=reqs.array_types.string, default={"joint_1", "joint_2", "joint_3"}, description="Joint names that are desired to be moved with a trapezoidal velocity profile. The size must coincide with the size of target_joints", required=true, minItems = 1}),
    reqs.params.enum({name="units", type=reqs.enum_types.string, default="radians", description="Units to be used for specifying the joints", required=false, accepted_vals = {"degrees","radians"}}),
})

robot = reqs.robot_model({
    -- "tcp_frame",
    -- "forearm",
    --Add all frames that are required by the task specification
})


-- ========================================= PARAMETERS ===================================
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))

target_joints = param.get("target_joints")
robot_joints = param.get("robot_joints")


if #robot.robot_joints <= #target_joints then
    error("The number of target_joints (" .. tostring(#target_joints) .. ") must be less than or equal to the number of specified robot.robot_joints (" .. tostring(#robot.robot_joints) ..  ")")
end

if #robot_joints ~= #target_joints then
    error("The number of robot_joints (" .. tostring(#robot_joints) .. ") and the specified number of target_joints (" .. tostring(#target_joints) ..  "), both defined as a parameter, must coincide")
end


units = param.get("units")
-- print("units: ",units)
target_joints = reqs.adapt_to_units(target_joints,units)

-- ========================================= VELOCITY PROFILE ===================================

mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
current_jnt = {} -- current joint value


for i=1,#robot_joints do
    current_jnt[i]   = ctx:getScalarExpr(robot_joints[i])
    mp:addOutput( initial_value(time, current_jnt[i]), constant(target_joints[i]), maxvel, maxacc)
end




-- The following creates a trapezoidal velocity profile from the initial value of each angle, towards the target angle. It checks whether the joint is continuous or bounded,
-- and if it is continuous it takes the shortest path towards the angle. This makes the skill generic to any type of robot (e.g. the Kinova).
-- The old version used the above commented method, which is the one that is explained in the etasl tutorial.
-- print("kasdjaksjdhkahskdhajkshdjkahsjkhdkjahsdhka")
-- for i=1,#robot.robot_joints do
--     current_jnt[i]   = ctx:getScalarExpr(robot.robot_joints[i])
--     local theta_init = initial_value(time, current_jnt[i])
--     local theta_final_raw = target_joints[i]
--     print(theta_final_raw)
--     local difference_theta = cached(acos(cos(theta_init)*cos(theta_final_raw)+sin(theta_init)*sin(theta_final_raw))) --Shortest angle between two unit vectors (basic formula: 'cos(alpha) = dot(a,b)'. where a and b are two unit vectors)
--     local error_difference_theta = cached(acos(cos(theta_init + difference_theta)*cos(theta_final_raw)+sin(theta_init + difference_theta)*sin(theta_final_raw))) --Shortest angle computation also. If the sign is correct, it should be zero
--     local delta_theta = cached(conditional(error_difference_theta - constant(1e-5) ,constant(-1)*difference_theta,difference_theta)) --determines the proper sign to rotate the initial angle

--     local is_continuous = ctx:createInputChannelScalar("continuous_j"..i,0)--TODO: In the next release we will be able to obtain this directly from the urdf
--     local final_angle = cached(conditional(constant(-1)*abs(is_continuous),theta_final_raw,theta_init + delta_theta)) -- Only 0 is interpreted as bounded
--     mp:addOutput( theta_init, make_constant(final_angle) , maxvel, maxacc)
-- end

duration = get_duration(mp)
-- print(duration:value())

-- ========================= CONSTRAINT SPECIFICATION ========================

tgt         = {} -- target value
tracking_error = {}
for i=1,#robot_joints do
    tgt[i]        = get_output_profile(mp,i-1)
    Constraint{
        context=ctx,
        name="joint_trajectory_"..robot_joints[i],
        expr= current_jnt[i] - tgt[i] ,
        priority = 2,
        K=3
    };
    tracking_error[i] = current_jnt[i] - tgt[i]

    -- Constraint{
    --     context=ctx,
    --     name="joint_trajectory"..i,
    --     expr= current_jnt[i] - initial_value(time, current_jnt[i]),
    --     priority = 2,
    --     K=1
    -- };


    -- Constraint{
    --     context=ctx,
    --     name="joint_trajectory"..i,
    --     expr= current_jnt[i] - target_joints[i] ,
    --     priority = 2,
    --     K=1
    -- };

    -- Constraint{
    --     context=ctx,
    --     name="joint_trajectory"..i,
    --     expr= current_jnt[i]*time ,
    --     priority = 2,
    --     K=0
    -- };

    
end

--    Constraint{
--         context=ctx,
--         name="joint_trajectory6",
--         expr= current_jnt[6] - 20*3.1416/180 ,
--         priority = 2,
--         K=1
--     };

-- =================================== MONITOR TO FINISH THE MOTION ========================

Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-duration -constant(2)
}

-- Monitor {
--     context = ctx,
--     name    = "time_elapsed",
--     expr    = time,
--     upper   = 1.0,
--     actionname = "print",
--     argument = "addtional argument"
-- }




ctx:setOutputExpression("time",time)


for i=1,#robot.robot_joints do
    ctx:setOutputExpression("jpos"..i,ctx:getScalarExpr(robot.robot_joints[i]))
end


for i=1,#robot_joints do
    ctx:setOutputExpression("tracking_error_"..robot_joints[i],tracking_error[i])
end