require("context")
require("geometric")
-- worldmodel=require("worldmodel")

require("math")
reqs = require("task_requirements")

task_description = "This task specification allows to move the position of the end effector in cartesian space relative to the initial pose, while maintaining a constant orientation."

-- ========================================= PARAMETERS ===================================

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration [m/s^2]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="eq_r", description="Equivalent radius", default = 0.08, required=false}),
    reqs.params.scalar({name="error_pos_th", description="Position error threshold for monitoring [m]", default = 0.0005, required=false}),
    reqs.params.scalar({name="error_rot_th", description="Rotation error threshold for monitoring [rad]", default = 0.01, required=false}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=false}),
    reqs.params.array({name="delta_pos", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, description="3D array of distances [m] that the robot will move w.r.t. the starting position in the X,Y,Z coordinates w.r.t. robot base", required=true,minimum = -1.5, maximum=1.5,minItems = 3, maxItems = 3}),
    reqs.params.array({name="delta_euler", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, description="3D array of euler angles [rad] that the robot will move w.r.t. the starting orientation following RPY convention w.r.t the robot base", required=true,minimum = -6.28, maximum=6.28,minItems = 3, maxItems = 3}),
    reqs.params.enum({name="wrt_frame", type=reqs.enum_types.string, default="world_frame", description="Defines in which frame the dela_pos and delta_euler are defined.", required=true, accepted_vals = {"tcp_frame","world_frame"}}),

    -- reqs.params.string({name="controlled_link", description="Name of the URDF link (i.e. a frame) used to control the robot in cartesian space", default = "tool0", required=true}),
    -- reqs.params.string({name="base_link", description="Name of the URDF link (i.e. a frame) that defines which joints will be used to generate the motion. Only joints from this link towards the controlled link will be used in the motion.", default = "world", required=true}),
    -- reqs.params.string({name="wrt_link", description="Name of the URDF link (i.e. a frame) to define w.r.t. in which frame the delta_pos and delta_euler are defined", default = "world", required=true}),
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


-- robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
--     -- param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
--     -- "forearm"
--     "tcp_frame"
--     --Add all frames that are required by the task specification
-- })
-- robot_joints = robot.robot_joints
-- local robot_world_model = robot.urdfreader.readUrdf(robot.xmlstr,{})

-- local VLT = {}
-- local local_frames = robot_world_model:getExpressions(VLT,ctx,{tf_transform ={param.get("controlled_link"),param.get("base_link")},
--                                                         wrt_transform ={param.get("wrt_link"),param.get("base_link")}})

-- if local_frames["tf_transform"]== nil then
--     error("The transformation from `" .. param.get("controlled_link") .. "` to `" .. param.get("base_link") .. "` does not exist. Is likely that these link names where not defined in the robot model (e.g. URDF).`")
-- end

-- if local_frames["wrt_transform"]== nil then
--     error("The transformation from `" .. param.get("wrt_link") .. "` to `" .. param.get("base_link") .. "` does not exist. Is likely that these link names where not defined in the robot model (e.g. URDF).`")
-- end

-- task_frame = local_frames["tf_transform"]
-- wrt_frame = local_frames["wrt_transform"]

-- ========================================= PARAMETERS ===================================
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))
eqradius  = constant(param.get("eq_r"))

error_pos_th = constant(param.get("error_pos_th"))
error_rot_th = constant(param.get("error_rot_th"))

delta_pos = param.get("delta_pos")
delta_x   = constant(delta_pos[1])
delta_y   = constant(delta_pos[2])
delta_z   = constant(delta_pos[3])

delta_euler = param.get("delta_euler")
delta_roll   = constant(delta_euler[1])
delta_pitch   = constant(delta_euler[2])
delta_yaw   = constant(delta_euler[3])


-- =============================== INITIAL POSE ==============================

startpose = initial_value(time, task_frame)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- =============================== END POSE ==============================

if(param.get("wrt_frame") == "tcp_frame") then
    end_frame = startpose*frame(rot_x(delta_roll)*rot_y(delta_pitch)*rot_z(delta_yaw),vector(delta_x,delta_y,delta_z))
    endpos = origin(end_frame)
    endrot = rotation(end_frame)
else
    endpos    = origin(startpose) + vector(delta_x,delta_y,delta_z)
    endrot    = rot_x(delta_roll)*rot_y(delta_pitch)*rot_z(delta_yaw)*rotation(startpose)
 end

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
--Error calculation
local rot_vec = getRotVec(inv(rotation(task_frame))*endrot)
local diff_rot_error
local angle_error
diff_rot_error, angle_error = normalize(rot_vec)

local error_pos = norm(endpos-origin(task_frame))
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