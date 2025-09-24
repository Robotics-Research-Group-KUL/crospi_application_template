require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")

reqs = require("task_requirements")
 
task_description = "This task specification allows to a desired task frame to a desired pose in cartesian space."

-- ========================================= PARAMETERS ===================================
 
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration [m/s^2]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="eq_r", description="Equivalent radius [m]", default = 0.08, required=false}),
    reqs.params.scalar({name="error_pos_th", description="Position error threshold for monitoring [m]", default = 0.0002, required=false}),
    reqs.params.scalar({name="error_rot_th", description="Rotation error threshold for monitoring [rad]", default = 0.01, required=false}),
    reqs.params.string({name="task_frame_world", description="Task frame from world to tcp", default = "tcp_frame", required=false}),
    reqs.params.string({name="task_frame_mav", description="Task frame from mav to tcp", default = "tcp_frame_wrt_mav", required=false}),
    reqs.params.array({name="desired_pose", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, description="Array with the desired pose of the task frame in [x,y,z,qx,qy,qz,qw]", required=true, minItems = 7, maxItems = 7}),
    reqs.params.string({name="mav_pose_wrt_world", description="The specified task frame will be kept still relative to this base frame that will continuously adapt. This frame should come from a PoseInputHandler or a TFInputHandler", default = "mav_pose_wrt_world", required=true}),
})

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame_world"), --The frame is selected as a parameter, to make the skill even more reusable
    param.get("task_frame_mav"), --The frame is selected as a parameter, to make the skill even more reusable
    "mav_base_link"
    -- "forearm"
    -- "tcp_frame"
    --Add all frames that are required by the task specification
})

robot_joints = robot.robot_joints
task_frame_mav  = robot.getFrame(param.get("task_frame_mav"))
mav_base_link  = robot.getFrame("mav_base_link")
tool0_cables_mav  = robot.getFrame("tool0_cables_mav")


mav_pose_wrt_world   = ctx:createInputChannelFrame(param.get("mav_pose_wrt_world"))

-- task_frame_world_without_mav_feedforward = make_constant(mav_base_link)*tool0_cables_mav
task_frame_world_without_mav_feedforward = mav_pose_wrt_world*tool0_cables_mav

task_frame_world = task_frame_world_without_mav_feedforward
-- task_frame_world = robot.getFrame(param.get("task_frame_world"))

base_frame   = ctx:createInputChannelFrame("board_pose")
joystick_input   = ctx:createInputChannelTwist("joystick_input")


-- ========================================= PARAMETERS ===================================
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))
eqradius  = constant(param.get("eq_r"))

error_pos_th = constant(param.get("error_pos_th"))
error_rot_th = constant(param.get("error_rot_th"))

desired_pose = param.get("desired_pose")

x_coordinate   = constant(desired_pose[1])
y_coordinate   = constant(desired_pose[2])
z_coordinate   = constant(desired_pose[3])
q_i            = constant(desired_pose[4])
q_j            = constant(desired_pose[5])
q_k            = constant(desired_pose[6])
q_real         = constant(desired_pose[7])

-- compute orientation from quaternion
quat = quaternion(q_real,vector(q_i,q_j,q_k))
target_R = toRot(quat)
target_P = vector(x_coordinate,y_coordinate,z_coordinate)
target_pose = frame(target_R, target_P)

-- =============================== INITIAL POSE ==============================
startpose = initial_value(time, task_frame_world)
startpos  = origin(startpose)
startrot  = rotation(startpose)

startpos_x = coord_x(startpos)

-- =========================== VELOCITY PROFILE ============================================
eps=constant(1E-14)
function close_to_zero( e, yes_expr, no_expr)
    return cached( conditional( e - eps, no_expr, conditional( -e+eps,  no_expr, yes_expr)) )
end

function normalize( v )
    n  = cached( norm(v) )
    vn = cached( close_to_zero(n, vector(constant(1),constant(0),constant(0)), v/n) )
    return vn,n
end

-- compute distances for displacements and rotations:
diff                    = cached(target_P-startpos)
diff, distance          = normalize( diff )

diff_rot                = cached(  getRotVec( inv(startrot)*target_R )) -- eq. axis of rotation for rotation from start to end:w
diff_rot, angle         = normalize( diff_rot )


-- plan trapezoidal motion profile in function of time:
mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
mp:addOutput(constant(0), distance, maxvel, maxacc)
mp:addOutput(constant(0), angle*eqradius, maxvel, maxacc)
d  = get_output_profile(mp,0)            -- progression in distance
r  = get_output_profile(mp,1)/eqradius   -- progression in distance_rot (i.e. rot*eqradius)

-- =========================== TARGET POSE ============================================

-- targetpos = startpos + diff*d
-- targetrot = startrot*rotVec(diff_rot,r)

-- target    = frame(targetrot,targetpos)

target = startpose

-- ========================== CONSTRAINT SPECIFICATION =================================
Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(target)*task_frame_world,
    K       = 3,
    weight  = 1,
    priority= 2
}

-- This expression should go from mav to maira
print("Holaaa")
-- Constraint{
--     context  = ctx,
--     name     = "arm_workspace",
--     expr      = coord_x(origin(task_frame_mav)),
--     target_lower    = -0.1,
--     target_upper    = 0.1,
--     K        = 2,
--     weight   = 5,
--     priority = 2
-- }

-- ========================== PLATFORM CONSTRAINTS =================================
print("Hola1")
-- board_frame_wrt_world = frame(initial_value(time, rotation(mav_base_link))*rot_x(constant(math.pi/2))*rot_y(constant(10*math.pi/180))*rot_z(constant(10*math.pi/180)), origin(initial_value(time, mav_base_link)))

-- board_frame_wrt_world = initial_value(time,base_frame)*frame(rot_y(constant(10*math.pi/180)))
board_frame_wrt_world = base_frame

feature_variable_pitch = Variable{context=ctx, name="feature_variable_pitch", initial_value=0.0}
feature_variable_roll  = Variable{context=ctx, name="feature_variable_roll", initial_value=0.0}
print("Hola2")
rotation_control_mav = rotation(mav_base_link)*rot_y(feature_variable_roll)*rot_x(feature_variable_pitch)

-- Uncomment this to leave MAV orientation aligned with BOARD (vision needed!!)
Constraint{
    context  = ctx,
    name     = "maintain_mav_alignment_with_board",
    expr     = inv(rotation(board_frame_wrt_world))*rotation_control_mav,
    K        = 6,
    weight   = 10,
    priority = 2
};

-- Uncomment this to leave MAV theta constant
-- theta_mav = ctx:getScalarExpr("theta_mobile_joint")
-- initial_mav_theta = initial_value(time, theta_mav)
-- Constraint{
--     context  = ctx,
--     name     = "mav_theta",
--     expr     = theta_mav,
--     target   = initial_mav_theta,
--     K        = 2,
--     weight   = 5,
--     priority = 2
-- };

-- desired_vel_theta = coord_z(rotvel(joystick_input))*constant(0.3)

-- theta_mav = ctx:getScalarExpr("theta_mobile_joint")
-- Constraint{
--     context  = ctx,
--     name     = "mav_theta_vel_joystick",
--     expr     = theta_mav,
--     target   = desired_vel_theta*time,
--     K        = 0,
--     weight   = 5,
--     priority = 2
-- };

-- mav_base_wrt_board_transform = inv(make_constant(rotation(frame_test)))*rotation(mav_base_link)
-- diff_rot_mav = getRotVec(mav_base_wrt_board_transform)
-- diff_rot_mav, error_angle_mav         = normalize( diff_rot_mav )

-- Constraint{
--     context = ctx,
--     name = "maintain_board_alignment_with_mav",
--     expr = error_angle_mav,
--     target = constant(0),
--     K = 0.1,
--     weight = 10,
--     priority= 2
-- };


task_frame_inst_mobile_base = inv(make_constant(mav_base_link))*mav_base_link

function linear_weight(w_init,w_final,ind_var,ind_var_init, ind_var_final)
    local lin_weight = w_init - ((w_init-w_final)/(ind_var_final-ind_var_init))*(ind_var-ind_var_init)
    local weight = conditional(ind_var-ind_var_init,conditional(ind_var-ind_var_final,w_final,lin_weight),w_init)
    return weight
end
  
value_const = 0.1
variable_weight = linear_weight(constant(1) , constant(0.5), abs(coord_x(origin(task_frame_mav))),value_const-0.02, value_const)

linear_scale = constant(0.3)
desired_vel_x = coord_x(transvel(joystick_input))*linear_scale

-- sine_amplitude = constant(0.05)
-- sine_freq = 0.3
-- desired_vel_x = sine_amplitude*sin(2*3.1416*sine_freq*time)


Constraint{
    context = ctx,
    name    = "mav_x_velocity_constraint",
    expr    = coord_x(origin(task_frame_inst_mobile_base)),
    target  = desired_vel_x*time, -- Zero velocity constraint
    K       = 0,
    weight  = 2,
    priority= 2
};

-- =========================== MONITOR ============================================


--Error calculation
local rot_vec = getRotVec(inv(rotation(task_frame_world))*target_R)
local diff_rot_error
local angle_error
diff_rot_error, angle_error = normalize(rot_vec)

local error_pos = norm(target_P-origin(task_frame_world))
local error_orient = angle_error
local error = conditional(error_pos-error_pos_th, constant(0), constant(1))*conditional(error_orient-error_rot_th, constant(0), constant(1)) -- Returns zero if the error is too big

-- Monitor{
--     context=ctx,
--     name='finish_after_motion',
--     upper = 0.8, -- The error, that comes from a conditional, is binary (0 or 1)
--     actionname = "exit",
--     expr=error
-- }

-- Monitor{
--     context=ctx,
--     name='debug',
--     upper = 5,
--     actionname = "debug",
--     expr=time
-- }


ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame_world)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame_world)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame_world)))
