#!/bin/bash

# Check if a directory is provided as an argument
if [ -z "$1" ]; then
  echo "Usage: $0 <path_of_generated_json_schema_for_robotdriver>"
  exit 1
fi

# Directory containing etasl Lua files (passed as a command-line argument)
OUTPUT_SCHEMA_ROBOTDRIVER="$1/robotdriver.json"


# Get the install directory of the workspace from the package prefix
WORKSPACE_INSTALL_DIR="$(ros2 pkg prefix etasl_ros2 | awk -F'/install' '{print $1}')/install"

# Check that the install directory exists
if [ ! -d "$WORKSPACE_INSTALL_DIR" ]; then
  echo "Error: Install directory '$WORKSPACE_INSTALL_DIR' does not exist."
  exit 1
fi

# Find plugin.xml files (regular or symlinked), that are not broken, and match your base_class_type
mapfile -t plugin_folders < <(
  find "$WORKSPACE_INSTALL_DIR" -name "plugins.xml" \( -type f -o -type l \) \
    -exec test -e {} \; \
    -exec grep -lP 'base_class_type\s*=\s*"etasl::RobotDriver"' {} + |
    xargs -r -n1 dirname | sort -u
)

plugin_folders_with_schema=()
plugin_folders_without_schema=()

for dir in "${plugin_folders[@]}"; do
  if [[ -f "$dir/robotdriver.schema.json" ]]; then
    plugin_folders_with_schema+=("$dir")
  else
    plugin_folders_without_schema+=("$dir")
  fi
done
# mapfile -t plugin_folders < <(
# find "$WORKSPACE_INSTALL_DIR" -name "plugins.xml" \( -type f -o -type l \) \   #search recursively in the install directory only at plugins.xml files, include both normal files and symlinks
#     -exec test -e {} \; \ #exclude broken symlinks, i.e. test whether the file or target exists
#     -exec grep -lP 'base_class_type\s*=\s*"etasl::RobotDriver"' {} + |  #Search only the plugins with etasl::RobotDriver as the base class. Search using a Perl-style regex to allow \s* (flexible whitespace)
#     xargs -r -n1 dirname | sort -u  #Avoid duplicate folders with sort -u. The xargs -r avoids running the command if grep finds no matches 
# )
# OUTPUT_SCHEMA_ROBOTDRIVER="test_generated_file.json"

echo "Plugins with base class etasl::RobotDriver that contain schema robotdriver.schema.json:"
printf '  %s\n' "${plugin_folders_with_schema[@]}"

echo "Plugins with base class etasl::RobotDriver that DO NOT contain schema robotdriver.schema.json:"
printf '  %s\n' "${plugin_folders_without_schema[@]}"



if [ -f "$OUTPUT_SCHEMA_ROBOTDRIVER" ]; then
    #   Clear the file if it exists
    > "$OUTPUT_SCHEMA_ROBOTDRIVER"
fi


echo '{
    "$schema": "http://json-schema.org/draft-06/schema",
    "$id": "generated/robotdriver.json",
    "title": "Robotdriver",
    "description": "Robotdriver interfaces robot hardware with eTaSL and runs in a separate thread, communicating via shared memory for reduced latency",
    "oneOf": [
        {"$ref":"https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_json_schemas/raw/main/schemas/no_driver.json"},' >> "$OUTPUT_SCHEMA_ROBOTDRIVER"


for json_folder in "${plugin_folders_with_schema[@]}"; do
#   cat "$json_folder"/robotdriver.schema.json
    echo "        {\"\$ref\": \"$json_folder/robotdriver.schema.json\"}," >> "$OUTPUT_SCHEMA_ROBOTDRIVER"

done

# Remove the last comma from the last sub-schema entry
truncate -s-2 "$OUTPUT_SCHEMA_ROBOTDRIVER"

# Close the JSON structure
echo '
    ]
}' >> "$OUTPUT_SCHEMA_ROBOTDRIVER"


# echo "$OUTPUT_SCHEMA_ROBOTDRIVER"


# robotdriver_spec_string="["
# # Loop through each Lua file in the directory
# for robot_file_path in "$LUA_ROBOT_SPEC_DIR"/*.etasl.lua; do #extensions with .etasl.lua
#   # Check if any Lua files exist
#   echo "-------------------"
#   if [ -f "$robot_file_path" ]; then
#     echo "Generating JSON-SCHEMA file for task specification: $robot_file_path..."
#     filename_robot=$(basename "$robot_file_path")
#     # filename_robot_without_ext="${filename_robot%.lua}"   
#     robotdriver_spec_string="${robotdriver_spec_string}\"${LUA_ROBOT_SPEC_DIR_INTERPOLATE}/${filename_robot}\", "
#   else
#     echo "Error: No Lua files found in $LUA_ROBOT_SPEC_DIR"
#     exit 1
#   fi
# done

# if [ -f "$robot_file_path" ]; then
#   # truncate -s-2 "$robotdriver_spec_string"  # Remove the last comma from the last sub-schema entry
#   robotdriver_spec_string="${robotdriver_spec_string%,*}"
# fi

# robotdriver_spec_string="${robotdriver_spec_string} ]"










# beginning_of_json_schema="${beginning_of_json_schema_1} ${robot_spec_string} ${beginning_of_json_schema_2}"
# echo "$beginning_of_json_schema" > "$OUTPUT_SCHEMA_ROBOTDRIVER"
