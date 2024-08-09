require("context")
require("geometric")


local vars = {}

-- maxvel = ctx:createInputChannelScalar("maxvel",0.5)

function createInputJsonScalar(var_name, default_val)
    table.insert(vars, var_name)
    return ctx:createInputChannelScalar(var_name, default_val)
end

-- Later in your Lua script or in an appropriate place, write the vars to a file
function write_json_schema(filename)
    -- local JSON = require("JSON") -- Ensure you have a JSON library like json, dkjson or cjson. In this case just json is used
    local dkjson = require("dkjson") -- Ensure you have a JSON library like json, dkjson or cjson. In this case just json is used
    local schema = {
        type = "object",
        properties = {},
        required = {}
    }

    for _, var in ipairs(vars) do
        schema.properties[var] = { type = "number" }
        table.insert(schema.required, var)
    end

    local file = io.open(filename, "w")
    -- file:write(JSON:encode(schema))
    file:write(dkjson.encode(schema, { indent = true }))
    file:close()
end

-- max_vel = createInputJsonScalar("maxvel", 0.65)
-- max_acc = createInputJsonScalar("maxacc", 0.65)


-- Call this function to write the JSON schema to a file

-- SCRIPT_NAME = string.gsub(string.match(debug.getinfo(1, 'S').short_src, "[^/]+$"), '.lua', '')
-- print(SCRIPT_NAME)

-- write_json_schema(SCRIPT_NAME .. ".json")
-- print("hello")