#!/bin/bash

# Check if a directory is provided as an argument
if [ -z "$1" ]; then
  echo "Usage: $0 <path_of_generated_json_schemas>"
  exit 1
fi

# Get the install directory of the workspace from the package prefix
WORKSPACE_INSTALL_DIR="$(ros2 pkg prefix etasl_ros2 | awk -F'/install' '{print $1}')/install"

# Check that the install directory exists
if [ ! -d "$WORKSPACE_INSTALL_DIR" ]; then
  echo "Error: Install directory '$WORKSPACE_INSTALL_DIR' does not exist."
  exit 1
fi



#Find all plugin type available with the etasl:: namespace in all plugins.xml files
mapfile -t plugin_types < <(
  find "$WORKSPACE_INSTALL_DIR" -name "plugins.xml" \( -type f -o -type l \) \
    -exec test -e {} \; \
    -exec grep -oP '\btype\s*=\s*"\Ketasl::[^"]+' {} + | \
    sed 's/^.*etasl:://' | \
    sort -u
)

echo "Plugin types found:"
printf '%s\n' "${plugin_types[@]}"


#Function definition for generating JSON schema for a specific base class type of plugins

get_installed_schema_files() {
  local base_class_type="$1"
  local plugin_folders_with_schema=()
  local plugin_folders_without_schema=()
  local installed_schema_files=()
  local OUTPUT_SCHEMA="$2"
  local BEGINNING_OF_SCHEMA="$3"

  # Find plugin.xml files (regular or symlinked), that are not broken, and match the provided base_class_type
  mapfile -t plugin_folders < <(
    find "$WORKSPACE_INSTALL_DIR" -name "plugins.xml" \( -type f -o -type l \) \
      -exec test -e {} \; \
      -exec grep -lP "base_class_type\\s*=\\s*\"$base_class_type\"" {} + |
      xargs -r -n1 dirname | sort -u
  )

  # Extract plugin names for this base class type
mapfile -t plugin_types_for_class < <(
  for dir in "${plugin_folders[@]}"; do
    awk -v base="$base_class_type" '
      /<class / {
        if ($0 ~ "base_class_type=\""base"\"") {
          match($0, /type="etasl::[^"]+"/)
          if (RSTART) {
            type_str = substr($0, RSTART+6, RLENGTH-7)
            print type_str
          }
        }
      }
    ' "$dir/plugins.xml"
  done | sort -u
)


  #Print base_class_type
  echo "Base class type: $base_class_type"
  echo "Plugin folders found:"
  printf '%s\n' "${plugin_folders[@]}"

  # Separate plugin folders into those with and without schema files
  for dir in "${plugin_folders[@]}"; do
    if [[ -d "$dir/json_schemas" ]] && compgen -G "$dir/json_schemas/*.schema.json" > /dev/null; then
      plugin_folders_with_schema+=("$dir")
    else
      plugin_folders_without_schema+=("$dir")
    fi
  done

  echo "Plugin folders with schema found:"
  printf '%s\n' "${plugin_folders_with_schema[@]}"
  echo "Plugin folders without schema found:"
  printf '%s\n' "${plugin_folders_without_schema[@]}"

  # Iterate through plugin folders with schema files and collect installed schema files
  for dir in "${plugin_folders_with_schema[@]}"; do
    for schema_path in "$dir"/json_schemas/*.schema.json; do
      [[ -e "$schema_path" ]] || continue  # Skip if no match (safety)

      # Extract the basename without extension (e.g., robotdriver from robotdriver.schema.json)
      schema_name=$(basename "$schema_path" .schema.json)

      # Check if this schema corresponds to an installed plugin
      # if printf '%s\n' "${plugin_types[@]}" | grep -Fxq "$schema_name"; then
      if printf '%s\n' "${plugin_types_for_class[@]}" | grep -Fxq "$schema_name"; then
        installed_schema_files+=("$schema_path")
      fi
      echo "Plugin types for base class ${base_class_type}:"
      printf '%s\n' "${plugin_types_for_class[@]}"
    done
  done

  if [ -f "$OUTPUT_SCHEMA" ]; then
    #   Clear the file if it exists
    > "$OUTPUT_SCHEMA"
  fi

  echo "${BEGINNING_OF_SCHEMA}"  >> "$OUTPUT_SCHEMA"


  for schema_file in "${installed_schema_files[@]}"; do
    echo "        {\"\$ref\": \"$schema_file\"}," >> "$OUTPUT_SCHEMA"
  done


  # Remove the last comma from the last sub-schema entry
  truncate -s-2 "$OUTPUT_SCHEMA"

  # Close the JSON structure
  echo '
      ]
  }' >> "$OUTPUT_SCHEMA"

  echo "Installed plugin schemas found:"
  printf '%s\n' "${installed_schema_files[@]}"

  # Return the installed schema files
  # echo "${installed_schema_files[@]}"
}

# Generating SHEMA for RobotDriver
# Directory containing etasl Lua files (passed as a command-line argument)
OUTPUT_SCHEMA_ROBOTDRIVER="$1/robotdriver.json"
OUTPUT_SCHEMA_INPUTHANDLER="$1/inputhandler.json"
OUTPUT_SCHEMA_OUTPUTHANDLER="$1/outputhandler.json"

BEGINNING_OF_SCHEMA_ROBOTDRIVER='{
    "$schema": "http://json-schema.org/draft-06/schema",
    "$id": "generated/robotdriver.json",
    "title": "Robotdriver",
    "description": "Robotdriver interfaces robot hardware with eTaSL and runs in a separate thread, communicating via shared memory for reduced latency",
    "oneOf": [
        {"$ref":"https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_json_schemas/raw/main/schemas/no_driver.json"},'



get_installed_schema_files "etasl::RobotDriver" "${OUTPUT_SCHEMA_ROBOTDRIVER}"  "${BEGINNING_OF_SCHEMA_ROBOTDRIVER}"


# Generating SHEMA for InputHandlers
BEGINNING_OF_SCHEMA_INPUTHANDLER='{
    "$schema": "http://json-schema.org/draft-06/schema",
    "$id": "generated/inputhandler.json",
    "title": "Inputhandler",
    "description": "An inputhandler get information from the outside world and put it into eTaSL",
    "oneOf": ['



get_installed_schema_files "etasl::InputHandler" "${OUTPUT_SCHEMA_INPUTHANDLER}"  "${BEGINNING_OF_SCHEMA_INPUTHANDLER}"





echo "Generated JSON schema for RobotDriver at: $OUTPUT_SCHEMA_ROBOTDRIVER"