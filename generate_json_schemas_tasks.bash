#!/bin/bash

# Check if a directory is provided as an argument
if [ -z "$2" ]; then
  echo "Usage: $0 <directory_with_lua_files> <directory_to_save_individual_json_schemas> <path_to_etasl_robot_specification(optional)>" 
  exit 1
fi

# Directory containing etasl Lua files (passed as a command-line argument)
LUA_ETASL_DIR="$1"

# Directory containing robot specification (passed as a command-line argument)
JSON_SCHEMAS_DIR="$2"

# Directory containing robot specification (passed as a command-line argument)
LUA_ROBOT_FILE="$3"


# Check if the directory exists
if [ ! -d "$LUA_ETASL_DIR" ]; then
  echo "Error: Directory $LUA_ETASL_DIR does not exist."
  exit 1
fi

# ------------------Generate one JSON schema per task specification located in $LUA_ETASL_DIR---------------------

# command_string_robot="print('No robot specification defined')"
# # Check if the LUA_ROBOT_FILE exists
# if [ -f "$LUA_ROBOT_FILE" ]; then
#   command_string_robot="dofile('${LUA_ROBOT_FILE}')" # LUA_ROBOT_FILE was provided and therefore used before task specification
# else
#   echo "Error: No valid LUA_ROBOT_FILE robot specification file was provided: $LUA_ROBOT_FILE does not exist."

# fi
# echo $command_string_robot

# Loop through each Lua file in the directory
for lua_file_dir in "$LUA_ETASL_DIR"/*.etasl.lua; do #extensions with .etasl.lua
  # Check if any Lua files exist
  echo "-------------------"
  if [ -f "$lua_file_dir" ]; then
    echo "Generating JSON-SCHEMA file for task specification: $lua_file_dir..."
    filename=$(basename "$lua_file_dir")
    filename_without_ext="${filename%.lua}"
    # dir_luafile="/home/santiregui/ros2_ws/src/etasl_ros2_application_template/etasl/task_specifications/test/${lua_file_dir}"
    # command_string="require('etasl_parameters');${command_string_robot};dofile('${lua_file_dir}'); write_json_schema('${lua_file_dir}'); print('Finished generating file ${filename_without_ext}.json')"
    command_string="require('task_requirements');_LUA_FILEPATH_TO_GENERATE_JSON_SCHEMA='${JSON_SCHEMAS_DIR}';dofile('${lua_file_dir}'); print('Finished generating file ${filename_without_ext}.json')"

    # echo $command_string
    lua -e "${command_string}"
  else
    echo "Error: No Lua files found in $LUA_ETASL_DIR"
    exit 1
  fi
done


# ------------------Generate one JSON schema constant string whose elements correspond to the robot specifications located in base path of $LUA_ROBOT_FILE---------------------
ROBOT_PATH_DIR=$(dirname "$LUA_ROBOT_FILE")

enum_string="["
# Loop through each Lua file in the directory
for robot_file_path in "$ROBOT_PATH_DIR"/*.etasl.lua; do #extensions with .etasl.lua
  # Check if any Lua files exist
  echo "-------------------"
  if [ -f "$robot_file_path" ]; then
    echo "Generating JSON-SCHEMA file for task specification: $robot_file_path..."
    filename_robot=$(basename "$robot_file_path")
    # filename_robot_without_ext="${filename_robot%.lua}"   
    enum_string="${enum_string}\"${filename_robot}\", "
  else
    echo "Error: No Lua files found in $ROBOT_PATH_DIR"
    exit 1
  fi
done

if [ -f "$robot_file_path" ]; then
  # truncate -s-2 "$enum_string"  # Remove the last comma from the last sub-schema entry
  enum_string="${enum_string%,*}"
fi

enum_string="${enum_string} ]"


# ------------------Generate a main schema file that references all the previously generated schemas ---------------------
output_schema="../tasks-schema.json"


# Start building the main schema
beginning_of_json_schema_1='{
    "$schema":"http://json-schema.org/draft-04/schema",
    "$id":"test_task_instances.json",
    "title":"Tasks configuration",
    "type":"object",
    "description":"Schema for definition of eTaSL tasks",
    "properties": {
          "tasks": {
              "title":"Tasks",
              "description":"Tasks (i.e. instances of task specifications) with specific parameters based on the application at hand.",
              "type" : "array",
              "items" : {
                "type": "object",
                "properties": {
                    "name":{
                        "description":"Name of the task (unique to the task instance)",
                        "type":"string"
                      },
                      "robot_specification_file":{
                        "description":"Name of the etasl lua file containing the robot specification",
                        "enum": '

beginning_of_json_schema_2='
                      },
                    "parameters":{
                        "oneOf": ['
                        
beginning_of_json_schema="${beginning_of_json_schema_1} ${enum_string} ${beginning_of_json_schema_2}"


echo "$beginning_of_json_schema" > "$output_schema"

echo "Generating JSON-SCHEMA general file for defining instances of task specifications"
# Loop through each Lua file in the directory
for schemas_file_dir in "$PWD"/*.etasl.json; do #extensions with .etasl.json
  # Check if any json files exist
  echo "-------------------"
  if [ -f "$schemas_file_dir" ]; then

        # Read the content of the sub-schema
    # schema_content=$(cat "$schemas_file_dir")
    filename=$(basename "$schemas_file_dir")
    echo "                            {\"\$ref\": \"task_json_schemas/$filename\"}," >> "$output_schema"
  else
    echo "Error: No .etasl.json files found in $PWD"
    rm "$output_schema"
    exit 1
    # break
  fi
done

if [ -f "$schemas_file_dir" ]; then
  # Remove the last comma from the last sub-schema entry
  truncate -s-2 "$output_schema"
fi

# Close the JSON structure
echo '
                        ]
                    }
                },
                "required": ["name","robot_specification_file","parameters"]
            }
        }
    },
    "required": ["tasks"]
}' >> "$output_schema"

echo "JSON schema generated at $output_schema"
