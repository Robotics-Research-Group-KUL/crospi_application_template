require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")

require("etasl_parameters")

set_task_description("This task specification allows to move the position of the end effector in cartesian space relative to the initial pose, while maintaining a constant orientation.")

-- ========================================= PARAMETERS ===================================
markus_parameter    = createScalarParameter("markus_parameter" ,2, "this is my documentation")
