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
local xmlstr = urdfreader.loadFile(etasl_application_share_dir .. "/robot_description/urdf/kuka_iiwa/use_case_setup_iiwa.urdf")
local robot_worldmodel = urdfreader.readUrdf(xmlstr,{})
-- robot:writeDot("kuka_iiwa_robot.dot")
local VL = {}
local frames = robot_worldmodel:getExpressions(VL,ctx,{tcp_frame={'right_tool0','world'}})

M.frames= frames
M.xmlstr = xmlstr
M.robot_worldmodel = robot_worldmodel
M.urdfreader = urdfreader

return M

