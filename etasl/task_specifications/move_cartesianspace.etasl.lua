require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
-- require("etasl_parameters")
reqs = require("task_requirements")

task_description = "This task specification allows to move the position of the end effector in cartesian space relative to the initial pose, while maintaining a constant orientation."

-- ========================================= PARAMETERS ===================================
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="eq_r", description="Equivalent radius", default = 0.08, required=false}),
    reqs.params.string({name="task_frame", description="Frame used to control the robot in cartesian space", default = "tcp_frame", required=false}),
    reqs.params.array({name="delta_pos", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, description="3D array of distances [m] that the robot will move w.r.t. the starting position in the X,Y,Z coordinates", required=true,minimum = -3, maximum=3,minItems = 3, maxItems = 3}),
})

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

delta_pos = param.get("delta_pos")
delta_x   = constant(delta_pos[1])
delta_y   = constant(delta_pos[2])
delta_z   = constant(delta_pos[3])


-- =============================== INITIAL POSE ==============================

startpose = initial_value(time, task_frame)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- =============================== END POSE ==============================
endpos    = origin(startpose) + vector(delta_x,delta_y,delta_z)
endrot    = rotation(startpose)

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
diff                    = cached(endpos-startpos)
diff, distance          = normalize( diff )

diff_rot                = cached(  getRotVec( inv(startrot)*endrot )) -- eq. axis of rotation for rotation from start to end:w
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
Monitor{
        context=ctx,
        name='finish_after_motion',
        upper=0.0,
        actionname='exit',
        expr=time-get_duration(mp) - constant(0.1)
}

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
ctx:setOutputExpression("x_tcp",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tcp",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tcp",coord_z(origin(task_frame)))



-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(task_frame))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)