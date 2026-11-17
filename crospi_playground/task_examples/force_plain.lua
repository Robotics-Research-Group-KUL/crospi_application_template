-- ============================================================================================
--  Force control: push down on the table with a constant force (plain eTaSL)
-- ============================================================================================
--  The force/torque sensor of the simulated robot is available as the input channel "wrench_input":
--  wrench exerted by the end effector on the environment, expressed in tool0 (FT_sensor_frame).
--  The tool moves down (along -z of the world) until the measured force reaches the target, using the
--  same model as crospi's guarded_motion: a virtual contact stiffness K_F relates displacement and force.
--  Start above the table (Settings > Initial joints > "Above the table").

require("context")
require("geometric")
local reqs = require("task_requirements")

local robot = reqs.robot_model({"tcp_frame", "FT_sensor_frame"})
local tcp = robot.getFrame("tcp_frame")
local ft_frame = robot.getFrame("FT_sensor_frame")

local wrench = ctx:createInputChannelWrench("wrench_input")

-- force exerted by the tool, expressed in the robot base frame
local f_base = rotation(ft_frame) * force(wrench)
local fz = -coord_z(f_base)                       -- pushing down gives a positive value

local target = constant(5.0)                      -- [N]
local K_F = constant(2000)                        -- [N/m] assumed contact stiffness

local start = initial_value(time, tcp)
Constraint{
    context  = ctx,
    name     = "force_z",
    model    = K_F * (coord_z(origin(start)) - coord_z(origin(tcp))),  -- force predicted by the displacement
    meas     = fz,
    target   = target,
    K        = 4,
    weight   = 1,
    priority = 2,
}
Constraint{
    context  = ctx,
    name     = "keep_xy",
    expr     = coord_x(origin(tcp)) - coord_x(origin(start)),
    K = 4, weight = 1, priority = 2,
}
Constraint{
    context  = ctx,
    name     = "keep_y",
    expr     = coord_y(origin(tcp)) - coord_y(origin(start)),
    K = 4, weight = 1, priority = 2,
}
Constraint{
    context  = ctx,
    name     = "keep_orientation",
    expr     = inv(rotation(start)) * rotation(tcp),
    K = 4, weight = 1, priority = 2,
}

ctx:setOutputExpression("force_z", fz)
ctx:setOutputExpression("target", target)
ctx:setOutputExpression("tcp_z", coord_z(origin(tcp)))
