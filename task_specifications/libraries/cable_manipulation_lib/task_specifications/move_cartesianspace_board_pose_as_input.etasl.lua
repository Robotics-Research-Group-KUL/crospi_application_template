require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")

reqs = require("task_requirements")
 
task_description = "This task specification allows to a desired task frame to a desired pose w.r.t. board frame"

-- ========================================= PARAMETERS ===================================
 
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration [m/s^2]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="eq_r", description="Equivalent radius [m]", default = 0.08, required=false}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=false}),
    reqs.params.array({name="desired_pose", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, description="Array with the desired pose of the task frame in [x,y,z,qx,qy,qz,qw]", required=true, minimum = -1.5, maximum = 1.5, minItems = 7, maxItems = 7}),
})

robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    -- param.get("task_frame_world"), --The frame is selected as a parameter, to make the skill even more reusable
    -- param.get("task_frame_mav"), --The frame is selected as a parameter, to make the skill even more reusable
    -- param.get("FT_sensor_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    -- "forearm"
    -- "tcp_frame"
    "mav_base_link",
    "tool0_cables_mav"
    --Add all frames that are required by the task specification
})


T_world_board   = ctx:createInputChannelFrame("board_pose")
mav_base_link  = robot.getFrame("mav_base_link")
tool0_cables_mav  = robot.getFrame("tool0_cables_mav")

task_frame_world_without_mav_feedforward = make_constant(mav_base_link)*tool0_cables_mav

-- TODO: Change order of quaterinions in the skill.
-- TODO: Check what happens when quaterion is not valid.
-- TODO: Change tcp_2_tf.

robot_joints = robot.robot_joints
-- task_frame = robot.getFrame(param.get("task_frame"))
 
-- ========================================= PARAMETERS ===================================
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))
eqradius  = constant(param.get("eq_r"))


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
target_R_board = toRot(quat)
target_P_board = vector(x_coordinate,y_coordinate,z_coordinate)
T_board_setpoint = frame(target_R_board, target_P_board)


T_world_setpoint = T_world_board * T_board_setpoint

target_R = rotation(T_world_setpoint)
target_P = origin(T_world_setpoint)

-- =============================== INITIAL POSE ==============================
startpose = initial_value(time, task_frame_world_without_mav_feedforward)
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
    expr    = inv(target)*task_frame_world_without_mav_feedforward,
    K       = 3,
    weight  = 1,
    priority= 2
}

-- =========================== MONITOR ============================================
Monitor{
        context=ctx,
        name='finish_after_motion',
        upper=0.0,
        actionname='exit',
        expr=time-get_duration(mp) - constant(2)
}

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
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame_world_without_mav_feedforward)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame_world_without_mav_feedforward)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame_world_without_mav_feedforward)))



-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(task_frame_world_without_mav_feedforward)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(task_frame_world_without_mav_feedforward)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(task_frame_world_without_mav_feedforward)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(task_frame_world_without_mav_feedforward))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)