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

-- ========================================= PARAMETERS ===================================
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))
eqradius  = constant(param.get("eq_r"))

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


-- ========================================== Imports utils_ts ============================
-- The following is done because utils_ts is a file of the library and not of the application ROS2 package.
local script_dir = debug.getinfo(1, "S").source:match("@(.*)/")
package.path = script_dir .. "/utilities/?.lua;" .. package.path  -- Add it to package.path
local utils_ts = require("utils_ts")

-- =========================== VELOCITY PROFILE ============================================
progress_variable = Variable{context = ctx, name ='path_coordinate', vartype = 'feature', initial = 0.0}

eps=constant(1E-14)

diff_rot                = cached(  getRotVec( inv(startrot)*endrot )) -- eq. axis of rotation for rotation from start to end:w
diff_rot, angle         = utils_ts.normalize( diff_rot )

r_inst = angle*progress_variable
-- =========================== VELOCITY PROFILE ============================================
d_time = constant(0)
mt=constant(1)

A = conditional(time-d_time,constant(1),constant(0))

s_initial = constant(0)
s_final = constant(1)
s_vel_motion_profile  = utils_ts.trap_velprofile( maxvel , maxacc , constant(0.0) , constant(1.0),  progress_variable)
s_velocity = conditional( time-d_time , s_vel_motion_profile , constant(0) )

Constraint{
	context=ctx,
	name = "vel_prof",
	expr = progress_variable - s_velocity*time,
	weight = 2,
	priority = 2,
	K = 0
};

Constraint{
	context=ctx,
	name = "reaching_vel_max",
	expr = progress_variable - maxvel*time,
	weight = 0.001,
	priority = 2,
	K = 0
};

Constraint{
	context=ctx,
	name = "s_min",
	expr = progress_variable,
	target_lower = s_initial,
	weight = 20,
	priority = 2,
	K = 4
};

Constraint{
	context=ctx,
	name = "s_max",
	expr = progress_variable,
	target_upper = s_final,
	weight = 20,
	priority = 2,
	K = 4
};

-- =========================== TARGET POSE ============================================
-- angle*eqradius
targetpos = startpos
targetrot = startrot*rotVec(diff_rot,r_inst)

target    = frame(targetrot,targetpos)

-- ========================== CONSTRAINT SPECIFICATION =================================
Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(target)*task_frame,
    K       = 3,
    weight  = 10,
    priority= 2
}


-- ========================= Velocity contraint joint ==================
base_joint = ctx:getScalarExpr(robot.robot_joints[1])
Constraint{
    context = ctx,
    name    = "low_vel_joint",
    expr    = base_joint - 0*time,
    K       = 0,
    weight  = 4,
    priority= 2
}

-- =========================== MONITOR ============================================
err = (s_final-progress_variable)
Monitor{
        context=ctx,
        name='finish_after_motion',
        lower=0.000,
        actionname='exit',
        expr=err
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