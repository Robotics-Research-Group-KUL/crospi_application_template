require("context")
require("geometric")
local ament = require("libamentlua")
-- worldmodel=require("worldmodel")
local urdfreader=require("urdfreader")

local M = {}

--
-- read robot model:
--

local etasl_application_share_dir = ament.get_package_share_directory("etasl_ros2_application_template")
local xmlstr = urdfreader.loadFile(etasl_application_share_dir .. "/robot_description/urdf/ur10/use_case_setup_ur10.urdf")
local robot_worldmodel = urdfreader.readUrdf(xmlstr,{})
-- robot:writeDot("ur10_robot.dot")
local VL = {}
local frames = robot_worldmodel:getExpressions(VL,ctx,{tcp_frame={'tool0','base_link'}})

M.frames= frames
M.xmlstr = xmlstr
M.robot_worldmodel = robot_worldmodel
M.urdfreader = urdfreader

return M