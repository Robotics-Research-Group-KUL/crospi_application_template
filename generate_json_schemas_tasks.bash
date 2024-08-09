#!/bin/bash

# Check if a directory is provided as an argument
if [ -z "$1" ]; then
  echo "Usage: $0 <directory_with_lua_files> <path_to_etasl_robot_specification(optional)>" 
  exit 1
fi

# Directory containing etasl Lua files (passed as a command-line argument)
LUA_ETASL_DIR="$1"

# Directory containing robot specification (passed as a command-line argument)
LUA_ROBOT_DIR="$2"

# Check if the directory exists
if [ ! -d "$LUA_ETASL_DIR" ]; then
  echo "Error: Directory $LUA_ETASL_DIR does not exist."
  exit 1
fi

command_string_robot="print('empty command')"
# Check if the LUA_ROBOT_DIR exists
if [ -f "$LUA_ROBOT_DIR" ]; then
  command_string_robot="dofile('${LUA_ROBOT_DIR}')" # LUA_ROBOT_DIR was provided and therefore used before task specification
else
  echo "Error: No valid LUA_ROBOT_DIR robot specification file was provided: $LUA_ROBOT_DIR does not exist."

fi
echo $command_string_robot

# Loop through each Lua file in the directory
for lua_file_dir in "$LUA_ETASL_DIR"/*.etasl.lua; do #extensions with .etasl.lua
  # Check if any Lua files exist
  echo "-------------------"
  if [ -f "$lua_file_dir" ]; then
    echo "Generating JSON-SCHEMA file for task specification: $lua_file_dir..."
    filename=$(basename "$lua_file_dir")
    filename_without_ext="${filename%.lua}"
    # dir_luafile="/home/santiregui/ros2_ws/src/etasl_ros2_application_template/etasl/task_specifications/test/${lua_file_dir}"
    command_string="require('etasl_json_schema_generator');${command_string_robot};dofile('${lua_file_dir}'); write_json_schema('${filename_without_ext}.json'); print('Finished generating file ${filename_without_ext}.json')"
    
    # echo $command_string
    lua -e "${command_string}"
  else
    echo "No Lua files found in $LUA_ETASL_DIR"
    break
  fi
done