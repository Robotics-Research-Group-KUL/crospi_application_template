-- ============================================================================================
--  Hello eTaSL: move the robot to a joint configuration
-- ============================================================================================
--  Each joint is driven to its target with a first-order behaviour  d(q - q_target)/dt = -K (q - q_target).
--  The task finishes (monitor with action "exit") when the total error is below 0.001 rad.
--  Try: change the target, the gain K or the priority/weight of the constraints and press Run again.

require("context")
require("geometric")
local reqs = require("task_requirements")

-- Robot model of the settings (URDF + robot specification of crospi)
local robot = reqs.robot_model({"tcp_frame"})
local tcp = robot.getFrame("tcp_frame")

local target_deg = {-90, -100, 110, -100, -90, 0}

local total_error = constant(0)
for i, name in ipairs(robot.robot_joints) do
    local q = ctx:getScalarExpr(name)
    local q_target = constant(target_deg[i] * math.pi / 180)
    Constraint{
        context  = ctx,
        name     = "joint_" .. i,
        expr     = q - q_target,
        K        = 1.0,
        weight   = 1,
        priority = 2,
    }
    total_error = total_error + abs(q - q_target)
    ctx:setOutputExpression("q" .. i, q)
end

-- outputs are plotted in the "Plots" tab
ctx:setOutputExpression("error", total_error)
ctx:setOutputExpression("tcp_z", coord_z(origin(tcp)))

Monitor{
    context    = ctx,
    name       = "target_reached",
    expr       = total_error,
    lower      = 0.001,        -- triggered when the expression goes below this value
    actionname = "exit",
}
