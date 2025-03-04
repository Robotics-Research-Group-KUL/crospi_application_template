require("context")
require("geometric")
require("libexpressiongraph_spline")
reqs = require("task_requirements")

task_description = [[
This task specification allows a task frame to follow a spline which is fitted based on a list of position points on a CSV file.
The CSV file must have 4 columns.
The CSV file must have normalized path variable (instead of time) that starts in 0 and ends in 1 (to indicate the progress), and the three remaining columns are X Y Z coordinates in that order.
]]

-- ========================================= PARAMETERS ===================================

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration [m/s^2]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.string({name="task_frame", description="Name of frame that will follow the spline", default = "tcp_frame", required=false}),
    reqs.params.string({name="csv_file_path", description="File path for CSV file containing the points that define the spline.", default = "$[etasl_ros2_application_template]/task_specifications/libraries/core_task_lib/task_specifications/misc/example_spline.csv", pattern=".*\\.csv$", required=true}),
})
-- spl:readPoints(etasl_application_share_dir.."/scripts/etasl/motion_models/hole_in_cylinder_contour.csv"," \t,",0)

-- ========================================= PARAMETERS ===================================
maxvel    = param.get("maxvel")
maxacc    = param.get("maxacc")
csv_file_path = param.get("csv_file_path")

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
})
task_frame = robot.getFrame(param.get("task_frame"))


-- =========================== DEGREE OF ADVANCEMENT =============================================

progress_variable = Variable{context = ctx, name ='path_coordinate', vartype = 'feature', initial = 0.0}
-- =============================== INITIAL POSE ==============================

startpose = initial_value(time, task_frame)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- ========================================== Imports utils_ts ============================
-- The following is done because utils_ts is a file of the library and not of the application ROS2 package.
local script_dir = debug.getinfo(1, "S").source:match("@(.*)/")
package.path = script_dir .. "/utilities/?.lua;" .. package.path  -- Add it to package.path
local utils_ts = require("utils_ts")

-- ========================================== GENERATE ORIENTATION PROFILE ============================

R_end = startrot*rot_z(constant(3.1416/2))

-- eq. axis of rotation for rotation from start to end:w
diff_rot                = cached(getRotVec( inv(startrot)*R_end ))
diff_rot, angle         = utils_ts.normalize( diff_rot )
--
r_inst = angle*progress_variable

-- ========================================== GENERATE PROFILES ============================

spl        = CubicSpline(0)
csv_file = utils_ts.path_interpolate(csv_file_path)
spl:readPoints(csv_file," \t,",0)
spl:setInput(progress_variable)
-- =========================== VELOCITY PROFILE ============================================
d_time = constant(0)
mt=constant(1)

A = conditional(time-d_time,constant(1),constant(0))

sa_n = 0
sb_n = 1

sa = constant(sa_n)
sb = constant(sb_n)
s_p_mp  = utils_ts.trap_velprofile( maxvel , maxacc , constant(0.0) , constant(1.0),  progress_variable)
s_p = conditional( time-d_time , s_p_mp , constant(0) )


Constraint{
	context=ctx,
	name = "vel_prof",
	expr = progress_variable - s_p*time,
	weight = A*constant(20),
	priority = 2,
	K = constant(0)
};

Constraint{
	context=ctx,
	name = "reaching_vel_max",
	expr = progress_variable - maxvel*time,
	weight = A*constant(0.001),
	priority = 2,
	K = constant(0)
};

Constraint{
	context=ctx,
	name = "s_min",
	expr = progress_variable,
	target_lower = sa,
	weight = constant(20),
	priority = 2,
	K = constant(4)
};

Constraint{
	context=ctx,
	name = "s_max",
	expr = progress_variable,
	target_upper = sb,
	weight = constant(20),
	priority = 2,
	K = constant(4)
};

-- ========================= FOLLOW GENERATED POSE ========================


e_x  = getSplineOutput(spl,0)
e_y  = getSplineOutput(spl,1)
e_z  = getSplineOutput(spl,2)



p_c= startpos + vector(e_x,e_y,e_z)

targetpos = p_c
targetrot = startrot*rotVec(diff_rot, r_inst)

target    = frame(targetrot,targetpos)

Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(target)*task_frame,
    weight  = constant(10),
    K       = constant(4),
}

-- =================================== MONITOR TO FINISH THE MOTION ========================

err = (sb-progress_variable)
Monitor{
        context=ctx,
        name='finish_after_motion',
        lower=0.000,
        actionname='exit',
        expr=err
}


-- ============================== OUTPUT PORTS===================================

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
ctx:setOutputExpression("progress_variable",progress_variable)