import json
# import requests
from jsonschema import validate, ValidationError

from typing import List, Callable, Union, Type

# def load_schema(schema_reference):
#     if schema_reference.startswith("http://") or schema_reference.startswith("https://"):
#         response = requests.get(schema_reference)
#         schema = response.json()
#     else:
#         with open(schema_reference, 'r') as schema_file:
#             schema = json.load(schema_file)
#     return schema

class param_manager():
    def __init__(self, json_file_name) -> None:
        with open(json_file_name, 'r') as json_file:
            self._parameters = json.load(json_file)

        # Function to find a task by its name
    def get_task(self, task_name: str)-> dict:
        for task in self._parameters.get("tasks", []):
            if task.get("name") == task_name:
                return task
        # print(f"Task with name '{task_name}' was not found.")
        raise Exception(f"Task with name '{task_name}' was not found.")
    
    def set_external_param(self, ):
    
    
    # with open('task_configuration/nested_sequence_example_etasl.json', 'r') as json_file:
    #         jsonfile = json.load(json_file)


# # Load JSON data from a file


# # Example usage
# task_name = "example_4"
# task = find_task_by_name(jsonfile, task_name)

# if task:
#     print(f"Found task: {task}")
#     # Access specific properties of the found task
#     robot_specification_file = task.get("robot_specification_file")
#     parameters = task.get("parameters")
    
#     print(f"robot_specification_file: {robot_specification_file}")
#     print(f"parameters: {parameters}")
# else:
#     print(f"Task with name '{task_name}' not found.")


# # print(type(jsonfile))




# # jsonfile["tasks"][0]["name"] #I need to make the tasks an array of objects with name

# # print(jsonfile)