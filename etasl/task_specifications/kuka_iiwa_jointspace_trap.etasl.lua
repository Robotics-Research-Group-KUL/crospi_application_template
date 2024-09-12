require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
require("etasl_parameters")

-- ========================================= PARAMETERS ===================================
maxvel    = createScalarParameter("maxvel" ,0.1, "Maximum velocity rad/s")
maxacc    = createScalarParameter("maxacc" , 0.1, "Maximum acceleration rad/s^2")

-- end_j = { 31/180*math.pi, 72/180*math.pi, -25/180*math.pi, -77/180*math.pi, -25/180*math.pi, 50/180*math.pi,  43/180*math.pi }

joint_1    = createScalarParameter("joint_1" ,0.0, "Target angle for joint_1 in radians")
joint_2    = createScalarParameter("joint_2" ,0.0, "Target angle for joint_2 in radians")
joint_3    = createScalarParameter("joint_3" ,0.0, "Target angle for joint_3 in radians")
joint_4    = createScalarParameter("joint_4" ,0.0, "Target angle for joint_4 in radians")
joint_5    = createScalarParameter("joint_5" ,0.0, "Target angle for joint_5 in radians")
joint_6    = createScalarParameter("joint_6" ,0.0, "Target angle for joint_6 in radians")
joint_7    = createScalarParameter("joint_7" ,0.0, "Target angle for joint_7 in radians")
end_j = { joint_1, joint_2,joint_3,joint_4,joint_5,joint_6, joint_7}
-- ========================================= VELOCITY PROFILE ===================================

mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
current_jnt = {} -- current joint value




for i=1,#robot_joints do
    current_jnt[i]   = ctx:getScalarExpr(robot_joints[i])
    mp:addOutput( initial_value(time, current_jnt[i]), end_j[i], maxvel, maxacc)
end




-- The following creates a trapezoidal velocity profile from the initial value of each angle, towards the target angle. It checks whether the joint is continuous or bounded,
-- and if it is continuous it takes the shortest path towards the angle. This makes the skill generic to any type of robot (e.g. the Kinova).
-- The old version used the above commented method, which is the one that is explained in the etasl tutorial.
-- print("kasdjaksjdhkahskdhajkshdjkahsjkhdkjahsdhka")
-- for i=1,#robot_joints do
--     current_jnt[i]   = ctx:getScalarExpr(robot_joints[i])
--     local theta_init = initial_value(time, current_jnt[i])
--     local theta_final_raw = end_j[i]
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
for i=1,#robot_joints do
    tgt[i]        = get_output_profile(mp,i-1)
    Constraint{
        context=ctx,
        name="joint_trajectory"..i,
        expr= current_jnt[i] - tgt[i] ,
        priority = 2,
        K=4
    };

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
    --     expr= current_jnt[i] - end_j[i] ,
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
        expr=time-duration +constant(0.2)
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

for i=1,#robot_joints do
    ctx:setOutputExpression("jpos"..i,current_jnt[i])
end
