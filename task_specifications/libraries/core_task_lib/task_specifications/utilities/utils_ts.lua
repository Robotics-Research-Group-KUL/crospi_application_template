require("context")
require("geometric")
require("math")


local M = {}


local function trap_velprofile(maxvel,maxacc,s_start,s_end, s_progress_var)
    local sc = conditional( constant(0.5)-maxvel*maxvel/(constant(2)*maxacc) , maxvel*maxvel/(constant(2)*maxacc), constant(0.5) )
    local s_p_3_4   = conditional(s_end-s_progress_var , make_constant( sqrt( constant(2)*maxacc*(s_end-s_progress_var) ) ) , constant(0))
    local s_p_2_4   = conditional( (s_end-sc)-s_progress_var , maxvel , s_p_3_4)
    local s_p_1_4   = conditional((sc+s_start)-s_progress_var , make_constant( sqrt( constant(2)*maxacc*(s_progress_var-s_start) ) ) , s_p_2_4)
    local s_p_0_4 = conditional(s_start-s_progress_var , constant(0) , s_p_1_4)

  return s_p_0_4
end

-- auxiliary functions:
local function close_to_zero( e, yes_expr, no_expr)
    local eps=constant(1E-14)
    return cached( conditional( e - eps, no_expr, conditional( -e+eps,  no_expr, yes_expr)) )
end
-- returns normalized vector and norm (taking into account that vector can be zero or very small)
local function normalize( v )
    local n  = cached( norm(v) )
    local vn = cached( close_to_zero(n, vector(constant(1),constant(0),constant(0)), v/n) )
    return vn,n
end

local function dead_zone(sign_0,dead_val)
   local sign = conditional(abs(sign_0)-dead_val, sign_0 + conditional(sign_0, -dead_val, dead_val), constant(0))
   return sign
end

local function path_interpolate(path_formatted)
    local ament = require("libamentlua")

    local copy_path_formatted = path_formatted

    local package_name, rest_of_path = path_formatted:match("%$%[(.-)%]/(.*)")
    
    if package_name then
        local package_dir = ament.get_package_share_directory(package_name)
        return package_dir .."/".. rest_of_path
    else
        return copy_path_formatted
    end

end


-- export functions
M.trap_velprofile = trap_velprofile
M.normalize = normalize
M.dead_zone = dead_zone
M.path_interpolate = path_interpolate
return M
