--  Copyright (c) 2025 KU Leuven, Belgium
--
--  Author: Santiago Iregui
--  email: <santiago.iregui@kuleuven.be>
--
--  GNU Lesser General Public License Usage
--  Alternatively, this file may be used under the terms of the GNU Lesser
--  General Public License version 3 as published by the Free Software
--  Foundation and appearing in the file LICENSE.LGPLv3 included in the
--  packaging of this file. Please review the following information to
--  ensure the GNU Lesser General Public License version 3 requirements
--  will be met: https://www.gnu.org/licenses/lgpl.html.
-- 
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Lesser General Public License for more details.

require("context")
require("geometric")
local ament = require("libamentlua")

-- worldmodel=require("worldmodel")
local urdfreader=require("urdfreader")

local M = {}


--
-- read robot model:
--

local etasl_application_share_dir = ament.get_package_share_directory("crospi_application_template")
local xmlstr = urdfreader.loadFile(etasl_application_share_dir .. "/robot_models/urdf_models/robot_setups/kuka_iiwa/use_case_setup_iiwa.urdf")
local robot_worldmodel = urdfreader.readUrdf(xmlstr,{})
-- robot:writeDot("kuka_iiwa_robot.dot")
local VL = {}
local frames = robot_worldmodel:getExpressions(VL,ctx,{tcp_frame = {'right_tool0','world'}, FT_frame = {'right_ft_sensor_frame', 'world'}})

M.frames= frames
M.xmlstr = xmlstr
M.robot_worldmodel = robot_worldmodel
M.urdfreader = urdfreader

return M

