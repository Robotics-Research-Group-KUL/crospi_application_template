cmake_minimum_required(VERSION 3.8)
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
set(WORK_DIR ${CMAKE_SOURCE_DIR}/etasl/task_specifications)
# set(LUA_SCRIPT ${WORK_DIR}/etasl_json_schema_generator.lua)
set(LUA_SCRIPT ${WORK_DIR}/move_cartesianspace.etasl.lua)

# Add a custom target to generate the JSON schema
# add_custom_command(generate_json_schema
#     COMMAND lua ${LUA_SCRIPT}
#     WORKING_DIRECTORY ${WORK_DIR}
#     COMMENT "Generating JSON schema..."
# )

execute_process (
  COMMAND lua ${LUA_SCRIPT}
  WORKING_DIRECTORY ${WORK_DIR}
) 


# Ensure that the JSON schema is generated before building
# add_dependencies(etasl_ros2_application_template generate_json_schema)

install(DIRECTORY config_files coordination etasl launch robot_description 
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
