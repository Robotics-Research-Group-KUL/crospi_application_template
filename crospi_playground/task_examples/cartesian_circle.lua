-- ============================================================================================
--  Cartesian motion: draw a horizontal circle with the tool tip
-- ============================================================================================
--  The position of the tool tip follows a circle (radius r, period T) around its starting position,
--  while its orientation is kept constant. The circle starts with a smooth radial ramp.
--  The task never finishes: press Stop to end it.

require("context")
require("geometric")
local reqs = require("task_requirements")

local robot = reqs.robot_model({"tcp_frame"})
local tcp = robot.getFrame("tcp_frame")          -- pose of the tool tip w.r.t. the robot base

local r = constant(0.08)                         -- radius [m]
local T = constant(6.0)                          -- period [s]
local ramp = constant(2.0)                       -- duration of the radial ramp [s]

local start = initial_value(time, tcp)           -- pose at the start of the task (constant)
local w = constant(2 * math.pi) / T
local s = conditional(time - ramp, constant(1), time / ramp)   -- 0 -> 1 during the ramp
local desired = origin(start) + vector(s * r * (cos(w * time) - constant(1)), s * r * sin(w * time), constant(0))

Constraint{
    context  = ctx,
    name     = "follow_circle",
    expr     = origin(tcp) - desired,            -- vector constraint (3 scalar constraints)
    K        = 4,
    weight   = 1,
    priority = 2,
}
Constraint{
    context  = ctx,
    name     = "keep_orientation",
    expr     = inv(rotation(start)) * rotation(tcp),
    K        = 4,
    weight   = 1,
    priority = 2,
}

ctx:setOutputExpression("x", coord_x(origin(tcp)))
ctx:setOutputExpression("y", coord_y(origin(tcp)))
ctx:setOutputExpression("x_desired", coord_x(desired))
ctx:setOutputExpression("y_desired", coord_y(desired))
