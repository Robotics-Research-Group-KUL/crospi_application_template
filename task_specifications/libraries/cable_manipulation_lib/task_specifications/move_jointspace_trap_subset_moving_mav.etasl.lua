require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
reqs = require("task_requirements")

task_description = "Moves a subset of joints (in joint space) to a target pose specified using joint angles."

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity rad/s", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration rad/s^2", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxvel_mav", description="Maximum linear velocity for MAV m/s", default = 0.1, required=true, maximum = 0.3}),
    reqs.params.scalar({name="maxacc_mav", description="Maximum linear acceleration for MAV m/s^2", default = 0.1, required=true, maximum = 0.3}),
    reqs.params.scalar({name="target_x_coordinate_mav", description="X target coordinate (absolute) for MAV of mav_base_link w.r.t. the world frame", default = 0.4, required=true, minimum = 0.0, maximum = 2.0}),
    reqs.params.array({name="target_joints", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, description="Array with target angles. Its values correspond to the defined in the parameter robot_joints (in the same order)", required=true, minimum = -360, maximum=360, minItems = 1}),
    reqs.params.array({name="robot_joints", type=reqs.array_types.string, default={"joint_1", "joint_2", "joint_3"}, description="Joint names that are desired to be moved with a trapezoidal velocity profile. The size must coincide with the size of target_joints", required=true, minItems = 1}),
    reqs.params.enum({name="units", type=reqs.enum_types.string, default="radians", description="Units to be used for specifying the joints", required=false, accepted_vals = {"degrees","radians"}}),
})

robot = reqs.robot_model({
    "mav_base_link"
    -- "tcp_frame",
    -- "forearm",
    --Add all frames that are required by the task specification
})


-- ========================================= PARAMETERS ===================================
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))
maxvel_mav    = constant(param.get("maxvel_mav"))
maxacc_mav    = constant(param.get("maxacc_mav"))
target_x_coordinate_mav = constant(param.get("target_x_coordinate_mav"))

target_joints = param.get("target_joints")
robot_joints = param.get("robot_joints")

mav_base_link  = robot.getFrame("mav_base_link")

T_world_board   = ctx:createInputChannelFrame("board_pose")





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
eps=constant(1E-14)
function close_to_zero( e, yes_expr, no_expr)
    return cached( conditional( e - eps, no_expr, conditional( -e+eps,  no_expr, yes_expr)) )
end

function normalize( v )
    n  = cached( norm(v) )
    vn = cached( close_to_zero(n, vector(constant(1),constant(0),constant(0)), v/n) )
    return vn,n
end


mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
current_jnt = {} -- current joint value


for i=1,#robot_joints do
    current_jnt[i]   = ctx:getScalarExpr(robot_joints[i])
    mp:addOutput( initial_value(time, current_jnt[i]), constant(target_joints[i]), maxvel, maxacc)
end


mp:addOutput( initial_value(time, coord_x(origin(mav_base_link))), target_x_coordinate_mav, maxvel_mav, maxacc_mav)

maxvel_mav_theta = constant(0.1)
maxacc_mav_theta = constant(0.1)
initial_rot_mav = initial_value(time, rotation(mav_base_link))
target_R_mav = rotation(T_world_board)

diff_rot_mav                = cached(  getRotVec( inv(initial_rot_mav)*target_R_mav )) -- eq. axis of rotation for rotation from start to end:w
diff_rot_mav, angle_mav         = normalize( diff_rot_mav )
mp:addOutput(constant(0), angle_mav, maxvel_mav_theta, maxacc_mav_theta)



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


-- --------------------Constraints for MAV --------------------
-- board_frame_wrt_world = initial_value(time,T_world_board)*frame(rot_y(constant(10*math.pi/180)))
board_frame_wrt_world = T_world_board

feature_variable_pitch = Variable{context=ctx, name="feature_variable_pitch", initial_value=0.0}
feature_variable_roll  = Variable{context=ctx, name="feature_variable_roll", initial_value=0.0}
print("Hola2")
rotation_control_mav = rotation(mav_base_link)*rot_y(feature_variable_roll)*rot_x(feature_variable_pitch)

-- Constraint{
--     context = ctx,
--     name    = "mav_x_trapezoidal_profile",
--     expr    = coord_x(origin(task_frame_inst_mobile_base)),
--     target  = desired_vel_x*time, -- Zero velocity constraint
--     K       = 0,
--     weight  = 2,
--     priority= 2
-- };

target_x_mav        = get_output_profile(mp, #robot_joints) --get_output_profile is zero-index-based
Constraint{
    context     =ctx,
    name        ="mav_x_trapezoidal_profile",
    expr        = coord_x(origin(mav_base_link)) - target_x_mav,
    priority    = 2,
    weight      = 1,
    K           = 3
};


-- --------------------Constraints for MAV orientation --------------------

rot_mav_mp        = get_output_profile(mp, #robot_joints+1) --get_output_profile is zero-index-based
targetrot_mav = initial_rot_mav*rotVec(diff_rot_mav,rot_mav_mp)

-- Uncomment this to leave MAV orientation aligned with board following a trapezoidal motion provile (vision needed!!)
Constraint{
    context  = ctx,
    name     = "maintain_mav_alignment_with_board_trapezoidal",
    expr     = targetrot_mav*rotation_control_mav,
    K        = 3,
    weight   = 10,
    priority = 2
};

-- Uncomment this to leave MAV orientation aligned with BOARD (vision needed!!)
-- Constraint{
--     context  = ctx,
--     name     = "maintain_mav_alignment_with_board",
--     expr     = inv(rotation(board_frame_wrt_world))*rotation_control_mav,
--     K        = 3,
--     weight   = 10,
--     priority = 2
-- };

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