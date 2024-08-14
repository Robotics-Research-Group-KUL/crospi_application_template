import json
# import requests
from jsonschema import validate, ValidationError

def load_schema(schema_reference):
    if schema_reference.startswith("http://") or schema_reference.startswith("https://"):
        response = requests.get(schema_reference)
        schema = response.json()
    else:
        with open(schema_reference, 'r') as schema_file:
            schema = json.load(schema_file)
    return schema
    

# Load JSON data from a file
with open('task_configuration/nested_sequence_example_etasl.json', 'r') as json_file:
    jsonfile = json.load(json_file)

print(jsonfile)