#!/bin/bash

#  Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Santiago Iregui
#  email: <santiago.iregui@kuleuven.be>
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.

# Check if a directory is provided as an argument
if [ -z "$3" ]; then
  echo "Usage: $0 <directory_with_task_libraries> <path_to_etasl_robot_specification> <path_to_etasl_robot_specification_to_be_interpolated>" 
  exit 1
fi

# Directory containing all task specifications whose json schema is going to be generated automatically
TASK_LIBRARIES_DIR="$1"

# Directory that this script uses to find the robot specifications
LUA_ROBOT_SPEC_DIR="$2"

# Directory containing robot specification with format to be interpolated (passed as a command-line argument).
# where to find the robot specifications, in a format that can be interpolated, i.e. with "$[package_name]/..."
LUA_ROBOT_SPEC_DIR_INTERPOLATE="$3"

# package to be used for crospi references to the .etasl.lua references.
# should correspond to what is declared in package.xml in this directory
ROS2_PACKAGE="crospi_application_template"

# Check if the directory exists
if [ ! -d "$TASK_LIBRARIES_DIR" ]; then
  echo "Error: Directory $TASK_LIBRARIES_DIR does not exist."
  exit 1
fi

# ------------------Generate one JSON schema per task specification located in $TASK_LIBRARIES_DIR---------------------

for task_library in "$TASK_LIBRARIES_DIR"/*; do
  if [ -d "$task_library" ] && [ -f "$task_library/task_library.json" ]; then    
    # If the directory does not exist, create it:
    mkdir -p "$task_library/task_json_schemas" 
    task_library_name=$(basename "$task_library")
    # Deletes all json schemas first. This avoids having non-existing json schema files, e.g. when you change the name of a task specification:
    find "$task_library/task_json_schemas" -name "*.etasl.json" -type f -delete 

    # Loop through each .etasl.lua file in the directory:
    for lua_file in "$task_library"/task_specifications/*.etasl.lua; do
      if [ -f "$lua_file" ]; then                                                   
        echo "Generating JSON-SCHEMA file for task specification: $lua_file..."
        filename=$(basename "$lua_file")
        filename_without_ext="${filename%.lua}"

        # _GENERATE                    : if true generate schema instead of executing complete task description.        
        # _FILEPATH_TASK_LIBRARY_JSON : full path of the file that describes the library: name, version and description of the task library, needed for the task schema.
        # _URI_TASK_LUA               : to refer to lua task specification in the schema and files using the schema (can use $[..] directives of crospi)
        # _FILEPATH_TASK_SCHEMA_JSON  : full path of the file to be written with schema for parameters of this task
        
        FILEPATH_TASK_LIBRARY_JSON="${task_library}/task_library.json"
        URI_TASK_LUA="\$[${ROS2_PACKAGE}]/task_specifications/libraries/${task_library_name}/task_specifications/$filename"
        FILEPATH_TASK_SCHEMA_JSON="${task_library}/task_json_schemas/${filename_without_ext}.json"

        command_string="require('task_requirements');\
                       _GENERATE=true;\
                       _FILEPATH_TASK_LIBRARY_JSON='${FILEPATH_TASK_LIBRARY_JSON}';\
                       _URI_TASK_LUA='${URI_TASK_LUA}';\
                       _FILEPATH_TASK_SCHEMA_JSON='${FILEPATH_TASK_SCHEMA_JSON}';\
                       dofile('${lua_file}');\
                       print('Finished generating file')"
        lua -e "${command_string}"
      else
        echo "Error: No Lua files found in $TASK_LIBRARIES_DIR"
        exit 1
      fi
    done

  fi
done

# ------------------Generate one JSON schema constant string whose elements correspond to the robot specifications located in base path of $LUA_ROBOT_SPEC_DIR---------------------

robot_spec_string="["
# Loop through each Lua file in the directory
for robot_file_path in "$LUA_ROBOT_SPEC_DIR"/*.etasl.lua; do #extensions with .etasl.lua
  # Check if any Lua files exist
  echo "-------------------"
  if [ -f "$robot_file_path" ]; then
    echo "Generating JSON-SCHEMA file for task specification: $robot_file_path..."
    filename_robot=$(basename "$robot_file_path")
    # filename_robot_without_ext="${filename_robot%.lua}"   
    robot_spec_string="${robot_spec_string}\"${LUA_ROBOT_SPEC_DIR_INTERPOLATE}/${filename_robot}\", "
  else
    echo "Error: No Lua files found in $LUA_ROBOT_SPEC_DIR"
    exit 1
  fi
done

if [ -f "$robot_file_path" ]; then
  # truncate -s-2 "$robot_spec_string"  # Remove the last comma from the last sub-schema entry
  robot_spec_string="${robot_spec_string%,*}"
fi

robot_spec_string="${robot_spec_string} ]"


# ------------------Generate a main schema file that references all the previously generated schemas ---------------------
output_schema="../tasks-schema.json"


# Start building the main schema
beginning_of_json_schema_1='{
    "$schema":"http://json-schema.org/draft-06/schema",
    "$id":"task-schema.json",
    "title":"Schema for configuration of tasks",
    "type":"object",
    "description":"Schema that enables all etasl task specifications in all the libraries installed within this application package for the creation of tasks",
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
                        "description":"(optional) If overriding the default_robot_specification, provide the name of the etasl lua file containing the robot specification.",
                        "type": "string",
                        "pattern": ".*\\.lua$",
                        "examples": '

beginning_of_json_schema_2='
                      },
                    "task_specification":{
                        "oneOf": ['
                        
beginning_of_json_schema="${beginning_of_json_schema_1} ${robot_spec_string} ${beginning_of_json_schema_2}"


echo "$beginning_of_json_schema" > "$output_schema"

echo "Generating JSON-SCHEMA general file for defining instances of task specifications"


for task_libraries in "$TASK_LIBRARIES_DIR"/*; do
  if [ -d "$task_libraries" ] && [ -f "$task_libraries/task_library.json" ]; then

  # Loop through each Lua file in the directory
  for schemas_file_dir in "$task_libraries"/task_json_schemas/*.etasl.json; do #extensions with .etasl.json
    # Check if any json files exist
    echo "-------------------"
    if [ -f "$schemas_file_dir" ]; then

          # Read the content of the sub-schema
      # schema_content=$(cat "$schemas_file_dir")
      filename=$(basename "$schemas_file_dir")
      libname=$(basename "$task_libraries")
      echo "                            {\"\$ref\": \"libraries/$libname/task_json_schemas/$filename\"}," >> "$output_schema"
    else
      echo "Error: No .etasl.json files found in $PWD"
      rm "$output_schema"
      exit 1
      # break
    fi
  done

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
                "required": ["name","task_specification"]
            }
        }
    },
    "required": ["tasks"]
}' >> "$output_schema"

echo "JSON schema generated at $output_schema"
