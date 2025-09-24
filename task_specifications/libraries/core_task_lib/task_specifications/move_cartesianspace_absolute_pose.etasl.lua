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
    reqs.params.scalar({name="error_pos_th", description="Position error threshold for monitoring [m]", default = 0.0005, required=false}),
    reqs.params.scalar({name="error_rot_th", description="Rotation error threshold for monitoring [rad]", default = 0.01, required=false}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=false}),
    reqs.params.array({name="desired_pose", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, description="Array with the desired pose of the task frame in [x,y,z,qx,qy,qz,qw]", required=true, minItems = 7, maxItems = 7}),
})

-- TODO: Change order of quaterinions in the skill.
-- TODO: Check what happens when quaterion is not valid.
-- TODO: Change tcp_2_tf.

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    -- "forearm"
    -- "tcp_frame"
    --Add all frames that are required by the task specification
})
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))
 
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
startpose = initial_value(time, task_frame)
startpos  = origin(startpose)
startrot  = rotation(startpose)

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

targetpos = startpos + diff*d
targetrot = startrot*rotVec(diff_rot,r)

target    = frame(targetrot,targetpos)

-- ========================== CONSTRAINT SPECIFICATION =================================
Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(target)*task_frame,
    K       = 3,
    weight  = 1,
    priority= 2
}

-- =========================== MONITOR ============================================
--Error calculation
local rot_vec = getRotVec(inv(rotation(task_frame))*target_R)
local diff_rot_error
local angle_error
diff_rot_error, angle_error = normalize(rot_vec)

local error_pos = norm(target_P-origin(task_frame))
local error_orient = angle_error
local error = conditional(error_pos-error_pos_th, constant(0), constant(1))*conditional(error_orient-error_rot_th, constant(0), constant(1)) -- Returns zero if the error is too big

Monitor{
    context=ctx,
    name='finish_after_motion',
    upper = 0.8, -- The error, that comes from a conditional, is binary (0 or 1)
    actionname = "exit",
    expr=error
}

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