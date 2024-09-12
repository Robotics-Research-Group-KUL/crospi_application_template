require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")

require("etasl_parameters")

set_task_description("This task specification allows to move the position of the end effector in cartesian space relative to the initial pose, while maintaining a constant orientation.")

-- ========================================= PARAMETERS ===================================
maxvel    = createScalarParameter("maxvel" ,0.1, "Maximum velocity")
maxacc    = createScalarParameter("maxacc" , 0.1, "Maximum acceleration")
eqradius  = createScalarParameter("eq_r"   ,0.08, "Equivalent radius")
delta_x   = createScalarParameter("delta_x",0.0, "Distance [m] that the robot will move w.r.t. the starting position in the X coordinate")
delta_y   = createScalarParameter("delta_y",0.0, "Distance [m] that the robot will move w.r.t. the starting position in the Y coordinate")
delta_z   = createScalarParameter("delta_z",0.0, "Distance [m] that the robot will move w.r.t. the starting position in the Z coordinate")

-- ======================================== FRAMES ========================================

tf = ee
