cmake_minimum_required(VERSION 3.15)
project(etasl_ros2_application_template)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)





# Define the Lua script to be executed
set(WORK_DIR ${CMAKE_SOURCE_DIR}/task_specifications/libraries)
set(BASH_SCRIPT_TASK_SCHEMAS ${CMAKE_SOURCE_DIR}/generate_json_schemas_tasks.bash)
set(TASK_LIBRARIES_DIR ${CMAKE_SOURCE_DIR}/task_specifications/libraries) #Directory containing all task specifications whose json schema is going to be generated automatically
# set(JSON_SCHEMAS_DIR ${CMAKE_SOURCE_DIR}/task_specifications/task_json_schemas/) #Directory containing all task specifications whose json schema is going to be generated automatically
set(LUA_ROBOT_SPEC_DIR ${CMAKE_SOURCE_DIR}/robot_models/robot_specifications) #This is used to execute read the task specifications and avoid errors due to missing variables (e.g. task_frame or joint variables).
set(LUA_ROBOT_SPEC_DIR_INTERPOLATE "\$[${CMAKE_PROJECT_NAME}]/robot_models/robot_specifications") #This is used to execute read the task specifications and avoid errors due to missing variables (e.g. task_frame or joint variables).

set(BASH_SCRIPT_PLUGINS_SCHEMAS ${CMAKE_SOURCE_DIR}/generate_json_schemas_plugins.bash)
set(GENERATED_SETUP_SCHEMAS_DIR ${CMAKE_SOURCE_DIR}/schemas/generated) #Directory containing all task specifications whose json schema is going to be generated automatically

# [Santiago]: add_custom_target works and generates the task-schema.json, but the execute process doesn't
add_custom_target(
  TASKS_SCHEMA_JSON
  ALL # This target will be built by default (i.e. always)
  COMMAND bash ${BASH_SCRIPT_TASK_SCHEMAS} ${TASK_LIBRARIES_DIR} ${LUA_ROBOT_SPEC_DIR}  ${LUA_ROBOT_SPEC_DIR_INTERPOLATE} 2>&1 | tee ${CMAKE_SOURCE_DIR}/tasks-schema.log
  WORKING_DIRECTORY ${WORK_DIR}
  COMMENT "constructing tasks-schema.json file"
  VERBATIM #This allows to pass $[ in strings such as defined in LUA_ROBOT_SPEC_DIR_INTERPOLATE
)

#Generate the json schema files for the setup configuration (e.g. related to robotdrivers)
add_custom_target(
  PLUGIN_SCHEMA_JSON
  ALL # This target will be built by default (i.e. always)
  COMMAND bash ${BASH_SCRIPT_PLUGINS_SCHEMAS} ${GENERATED_SETUP_SCHEMAS_DIR} 2>&1 | tee ${CMAKE_SOURCE_DIR}/plugins-schema.log
  WORKING_DIRECTORY ${WORK_DIR}
  COMMENT "constructing json schema files for the setup configuration"
  VERBATIM #This allows to pass $[ in strings such as defined in LUA_ROBOT_SPEC_DIR_INTERPOLATE
)


# execute_process (
#   COMMAND bash ${BASH_SCRIPT_TASK_SCHEMAS} ${TASK_LIBRARIES_DIR} ${LUA_ROBOT_SPEC_DIR}
#   WORKING_DIRECTORY ${WORK_DIR} #Will change the current directory and thus the generated files will be saved here
#   COMMAND_ECHO STDOUT
# ) 

# this will be always called:



# Ensure that the JSON schema is generated before building
# add_dependencies(etasl_ros2_application_template generate_json_schema)

install(DIRECTORY applications task_specifications skill_specifications robot_models
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()

