import os
from ament_index_python.resources import get_resource, get_resources
# from ament_index_python.resources import get_resource_types
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
from typing import Optional
import json


def get_plugins_by_base_class(base_class_type: str) -> dict:
    plugins = {}
    # get_resource_types() --> Use this to see what you can put as argument of get_resources
    resource_names = get_resources('etasl_ros2__pluginlib__plugin')
    # print(f"Discovered pluginlib packages: {resource_names}")

    for package_name in resource_names:
        content, _ = get_resource('etasl_ros2__pluginlib__plugin', package_name)
        relative_xml_path = content.strip()

        # Remove the leading 'share/<package_name>/' from relative_xml_path if present
        prefix = f"share/{package_name}/"
        if relative_xml_path.startswith(prefix):
            relative_xml_path = relative_xml_path[len(prefix):]

        package_share_dir = get_package_share_directory(package_name)
        full_xml_path = os.path.join(package_share_dir, relative_xml_path)

        # print(f"Processing: {full_xml_path}")
        if not os.path.exists(full_xml_path):
            print(f"Warning: file not found: {full_xml_path}")
            continue

        tree = ET.parse(full_xml_path)
        root = tree.getroot()
        plugin_dir = os.path.dirname(full_xml_path)


        for class_elem in root.findall('class'):
            base_class = class_elem.get('base_class_type')
            if base_class == base_class_type:
                type_ = class_elem.get('type')
                name = type_.split("::")[-1]

                # Using type_ as the key (plugin identifier)
                plugins[name] = {
                    'type': type_,
                    'path': plugin_dir
                }

    return plugins

def find_plugin_json_schema(name: str, path_plugin: str) -> Optional[str]:
    """
    Fins a JSON schema for the plugin.
    
    :param name: Name of the plugin
    :param path: Path to the plugin
    :return: Path to the JSON schema file or None if not found
    """
    # Construct the target file name
    target_file = f"{name}.schema.json"
    path = os.path.join(path_plugin, 'json_schemas')

    # Check if the directory exists
    if not os.path.exists(path):
        print(f"Error: Path does not exist: {path}")
        return None
    json_schema_path = None
    if os.path.exists(os.path.join(path, target_file)): #Check if json schema file exists with the exact name of the plugin class and if its a symbolic link that is valid
        json_schema_path = os.path.join(path, target_file)
        #Check if symbolic link is broken
        print(f"--{target_file}: a json schema with the exact name (case-sensitive) was found!")
    else: #Check if json schema file exists with the case-insensitive name of the plugin class
        # Iterate through files in the directory to find a case-insensitive match
        for file in os.listdir(path):
            # print(f"hellooo:{file}")
            if file.lower() == target_file.lower():
                if os.path.exists(os.path.join(path, file)): #Check if symbolic link is broken
                    json_schema_path = os.path.join(path, file)
                    print(f"--{target_file}: a json schema with a non-perfect match of the plugin name (i.e. case-insensitive) was found:{file}!")
                    break
    
    #Check that json_schema_path is not none:
    if json_schema_path is None:
        print(f"Error: JSON schema file not found for {name} in {path}")
    
    return json_schema_path

def generate_json_schema(plugin_dict: dict, gen_schema_name: str, description: str, additional_refs: list = None) -> bool:
    """
    Generate a JSON schema for each plugin base type.
    
    :param plugin_dict: Dictionary containing plugin information
    :param gen_schema_name: Name of the generated schema
    :return: True if schema generation was successful, False otherwise
    """

    json_schema = {
        "$schema": "http://json-schema.org/draft-06/schema",
        "$id": f"generated/{gen_schema_name}",
        "title": gen_schema_name.split(".")[0].capitalize(),
        "description": description,
        "oneOf": []
    }

    # Add additional_refs to the "oneOf" array
    #Check that additional_refs is not None
    if not additional_refs is None:
        for ref in additional_refs:
            json_schema["oneOf"].append({"$ref": ref})
    print(f"Generating JSON schema for {gen_schema_name} with {len(plugin_dict)} the following detected installed plugins:")
    # for name, info in plugin_dict.items():
    #     print(f"- {info['type']}")
    # print(" ")
    
    for name, info in plugin_dict.items():
        # print(f"Name: {name}, Class type: {info['type']}, Path: {info['path']}")
        schema_path = find_plugin_json_schema(name, info['path'])
        if schema_path is None:
            print(f"WARNING: JSON schema not found for plugin {name} and therefore it has not being included in the generated schema file {gen_schema_name}.")
        else:
            # Add the schema path as a $ref entry
            json_schema["oneOf"].append({"$ref": schema_path})

    # Write the JSON schema to a file


    output_file = f"schemas/generated/{gen_schema_name}"
    try:
        with open(output_file, "w") as f:
            json.dump(json_schema, f, indent=4)
        print(f"JSON schema successfully generated and saved to {output_file}")
        print(" ")
        return True
    except Exception as e:
        print(f"Error: Failed to write JSON schema to {output_file}. Exception: {e}")
        return False


if __name__ == "__main__":
    # print("RobotDriver Plugins:")
    # plugins = get_plugins_by_base_class('etasl::RobotDriver')
    # for name, info in plugins.items():
    #     print(f"Name: {name}, Class type: {info['type']}, Path: {info['path']}")
    #     find_plugin_json_schema(name, info['path'])

    # # Example: for etasl::InputHandler base class
    # print("\nInputHandler Plugins:")
    # plugins = get_plugins_by_base_class('etasl::InputHandler')
    # for name, info in plugins.items():
    #     print(f"Name: {name}, Class type: {info['type']}, Path: {info['path']}")
    #     find_plugin_json_schema(name, info['path'])

    plugins = get_plugins_by_base_class('etasl::RobotDriver')
    description = "Robotdriver interfaces robot hardware with eTaSL and runs in a separate thread, communicating via shared memory for reduced latency"
    additional_refs = [
        "https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_json_schemas/raw/main/schemas/no_driver.json"
    ]
    success = generate_json_schema(plugins, "robotdriver.json", description, additional_refs)

    plugins = get_plugins_by_base_class('etasl::InputHandler')
    description = "An inputhandler get information from the outside world and put it into eTaSL, e.g. via ROS2 topics or other types of communication"
    success = generate_json_schema(plugins, "inputhandler.json", description)
    
    plugins = get_plugins_by_base_class('etasl::OutputHandler')
    description = "An outputhandler extract data from eTaSL and communicate this in some way to the outside world, e.g. writing to a ROS2 Topic or other types of communication"
    success = generate_json_schema(plugins, "outputhandler.json", description)

    