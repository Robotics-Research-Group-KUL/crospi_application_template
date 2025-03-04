import json
import jsonschema
import os

class ParamValidator:
    """
    A class to handle parameter validation based on JSON schema.
    """
        
    external_param = {"type": "string", "pattern": "^\\$blackboard(/[a-zA-Z0-9_\\-]+)+$"}

    enum_types = {
        "number": "number",
        "integer": "integer",
        "string": "string"
    }

    array_types = {
        "number": "number",
        "integer": "integer",
        "bool": "bool",
        "string": "string"
    }

    required_parameters = []

    @staticmethod
    def param_generic(spec):
        """
        Processes a dictionary specifying properties for a generic parameter.
        
        :param spec: dict containing parameter specifications:
            - name (str): Required. The name of the parameter.
            - description (str): Required. Description for documentation.
            - default: Required. Default value.
            - required (bool): Optional. Defaults to True.
        :return: dict representing a partial JSON schema for the parameter.
        """
        
        if not spec.get("name"):
            raise ValueError("The defined parameter requires a non-empty name")
        if not spec.get("description"):
            raise ValueError("The defined parameter requires a non-empty description")
        if "default" not in spec:
            raise ValueError("The defined parameter requires a default value")
        
        if spec.get("required", True):
            ParamValidator.required_parameters.append(spec["name"])
        
        param_schema = {
            spec["name"]: {
                "description": spec["description"] + ". \nSet as a string beginning with $blackboard such as $blackboard/output_param/my_task/my_param if the value is not yet known and thus will be set externally at runtime (only once) depending on e.g. the outcome of a previous action or the outcome of another module.",
                "examples": [f"$blackboard/output_param/my_task/{spec['name']}"]
            }
        }
        
        if "default" in spec:
            param_schema[spec["name"]]["default"] = spec["default"]
        
        return param_schema

    def param_numerical(spec, schema):
        """
        Processes numerical parameters, extending the generic parameter schema.
        
        :param spec: dict containing parameter specifications:
            - name (str): Required. Name of the parameter.
            - description (str): Required. Description for documentation.
            - default (number): Required. Default value.
            - required (bool): Optional. Defaults to True.
            - minimum (number): Optional. Minimum allowed value.
            - maximum (number): Optional. Maximum allowed value.
        :param schema: dict representing the partial JSON schema for the parameter.
        :return: tuple (updated schema, additional numerical constraints)
        """
        
        param_schema = schema
        tab_fields = {}
        
        if "minimum" in spec:
            if isinstance(spec["minimum"], (int, float)):
                tab_fields["minimum"] = spec["minimum"]
            else:
                raise ValueError(f"The minimum value specified for parameter {spec['name']} should be a number.")
        
        if "maximum" in spec:
            if isinstance(spec["maximum"], (int, float)):
                tab_fields["maximum"] = spec["maximum"]
            else:
                raise ValueError(f"The maximum value specified for parameter {spec['name']} should be a number.")
        
        if "default" in spec and not isinstance(spec["default"], (int, float)):
            raise ValueError(f"The default value specified for parameter {spec['name']} should be a number.")
        
        if "maximum" in spec and spec["default"] > spec["maximum"]:
            raise ValueError(f"The specified default value of the parameter {spec['name']} is above the specified maximum value")
        
        if "minimum" in spec and spec["default"] < spec["minimum"]:
            raise ValueError(f"The specified default value of the parameter {spec['name']} is below the specified minimum value")
        
        return param_schema, tab_fields

    def validate_allowed_specs(allowed_specs, spec, input_type):
        """
        Validates that only allowed keys are present in a parameter specification.
        
        :param allowed_specs: list of allowed keys.
        :param spec: dict containing parameter specifications.
        :param input_type: str describing the input type for logging purposes.
        :raises ValueError: if an invalid key is found.
        """
        for key_spec in spec.keys():
            if key_spec not in allowed_specs:
                allowed_specs_string = ", ".join(allowed_specs)
                raise ValueError(f"The input {key_spec} is not valid when defining parameters of type {input_type}. "
                                    f"The only allowed inputs are: {allowed_specs_string}")


    def p_scalar(spec):
        """
        Defines a scalar parameter.

        :param spec: A dictionary containing the specification of the parameter with specific keys and values:
            - 'name' (str): The name of the specified parameter. Required.
            - 'description' (str): The description of the specified parameter. Required.
            - 'default' (number): The default value of the specified parameter. Required.
            - 'required' (bool): Indicates if the parameter is required. Optional.
            - 'minimum' (number): Minimum value allowed. Optional.
            - 'maximum' (number): Maximum value allowed. Optional.
        :return: A dictionary containing a JSON schema defining the parameter.
        """
        allowed_specs = {"name", "description", "default", "required", "minimum", "maximum"}
        ParamValidator.validate_allowed_specs(allowed_specs, spec, "number")

        param_schema = ParamValidator.param_generic(spec)  # pre-fills generic data necessary by all types
        param_schema, tab_fields = ParamValidator.param_numerical(spec, param_schema)  # Fills fields required for all numbers (double and integers)
        tab_fields['type'] = "number"
        param_schema[spec['name']]['oneOf'] = [tab_fields, ParamValidator.external_param]
        
        return param_schema


    def p_int(spec):
        """
        Defines an integer parameter.

        :param spec: A dictionary containing the specification of the parameter with specific keys and values:
            - 'name' (str): The name of the specified parameter. Required.
            - 'description' (str): The description of the specified parameter. Required.
            - 'default' (number): The default value of the specified parameter. Required.
            - 'required' (bool): Indicates if the parameter is required. Optional.
            - 'minimum' (number): Minimum value allowed. Optional.
            - 'maximum' (number): Maximum value allowed. Optional.
        :return: A dictionary containing a JSON schema defining the parameter.
        """
        allowed_specs = {"name", "description", "default", "required", "minimum", "maximum"}
        ParamValidator.validate_allowed_specs(allowed_specs, spec, "integer")

        param_schema = ParamValidator.param_generic(spec)  # pre-fills generic data necessary by all types
        param_schema, tab_fields = ParamValidator.param_numerical(spec, param_schema)  # Fills fields required for all numbers (double and integers)
        tab_fields['type'] = "integer"
        param_schema[spec['name']]['oneOf'] = [tab_fields, ParamValidator.external_param]
        
        return param_schema


    def p_bool(spec):
        """
        Defines a boolean parameter.

        :param spec: A dictionary containing the specification of the parameter with specific keys and values:
            - 'name' (str): The name of the specified parameter. Required.
            - 'description' (str): The description of the specified parameter. Required.
            - 'default' (bool): The default value of the specified parameter. Required.
            - 'required' (bool): Indicates if the parameter is required. Optional.
        :return: A dictionary containing a JSON schema defining the parameter.
        """
        allowed_specs = {"name", "description", "default", "required"}
        ParamValidator.validate_allowed_specs(allowed_specs, spec, "boolean")

        param_schema = ParamValidator.param_generic(spec)  # pre-fills generic data necessary by all types
        if spec.get('default') is not None and not isinstance(param_schema[spec['name']]['default'], bool):
            raise ValueError(f"The default value specified for parameter {spec['name']} should be a boolean.")
        
        param_schema[spec['name']]['oneOf'] = [{'type': "boolean"}, ParamValidator.external_param]
        
        return param_schema


    def p_string(spec):
        """
        Defines a string parameter.

        :param spec: A dictionary containing the specification of the parameter with specific keys and values:
            - 'name' (str): The name of the specified parameter. Required.
            - 'description' (str): The description of the specified parameter. Required.
            - 'default' (str): The default value of the specified parameter. Required.
            - 'required' (bool): Indicates if the parameter is required. Optional.
            - 'pattern' (str): Indicates if the string should follow a regular expression (regex) pattern. Optional.
        :return: A dictionary containing a JSON schema defining the parameter.
        """
        allowed_specs = {"name", "description", "default", "required", "pattern"}
        ParamValidator.validate_allowed_specs(allowed_specs, spec, "string")

        param_schema = ParamValidator.param_generic(spec)  # pre-fills generic data necessary by all types
        if spec.get('default') is not None and not isinstance(param_schema[spec['name']]['default'], str):
            raise ValueError(f"The default value specified for parameter `{spec['name']}` should be a string.")
        
        if spec.get('pattern') and isinstance(spec['pattern'], str):
            try:
                re.match(spec['pattern'], "")
            except re.error:
                raise ValueError(f"The provided pattern for parameter `{spec['name']}` is not a valid regex pattern.")
            param_schema[spec['name']]['oneOf'] = [{'type': "string", 'pattern': spec['pattern']}, ParamValidator.external_param]
        else:
            param_schema[spec['name']]['oneOf'] = [{'type': "string"}, ParamValidator.external_param]

        return param_schema


    def p_enum(spec):
        """
        Defines an enum parameter.

        :param spec: A dictionary containing the specification of the parameter with specific keys and values:
            - 'name' (str): The name of the specified parameter. Required.
            - 'description' (str): The description of the specified parameter. Required.
            - 'default' (type): The default value of the specified parameter. Must belong to the accepted_vals and belong to the specified type. Required.
            - 'required' (bool): Indicates if the parameter is required. Optional.
            - 'type' (str): String specifying the type of enum. The allowed types are number, integer, and string. Required.
            - 'accepted_vals' (list): A list that contains the accepted values of the enum. Required.
        :return: A dictionary containing a JSON schema defining the parameter.
        """
        allowed_specs = {"name", "description", "default", "required", "type", "accepted_vals"}
        ParamValidator.validate_allowed_specs(allowed_specs, spec, "enum")

        param_schema = ParamValidator.param_generic(spec)  # pre-fills generic data necessary by all types

        if not spec.get('type'):
            raise ValueError(f"The type for enum parameter {spec['name']} was not specified. The valid types for are: number, integer, and string.")
        
        if spec['type'] not in ['number', 'integer', 'string']:
            raise ValueError(f"Incorrect specified type for enum parameter {spec['name']}. The only valid types for enum parameters are: number, integer, and string.")
        
        if not spec.get('accepted_vals'):
            raise ValueError(f"The accepted_vals for enum parameter {spec['name']} was not specified.")
        
        for value in spec['accepted_vals']:
            if not isinstance(value, spec['type']):
                raise ValueError(f"One of the specified accepted_vals of the enum {spec['name']} is not of the same type as the defined type.")
        
        if spec.get('default') and not isinstance(param_schema[spec['name']]['default'], spec['type']):
            raise ValueError(f"The default value specified for parameter {spec['name']} should be of the same type as the specified type.")

        if spec['default'] not in spec['accepted_vals']:
            raise ValueError(f"The specified default value of the enum {spec['name']} does not correspond to one of the specified accepted_vals")
        
        param_schema[spec['name']]['oneOf'] = [{'enum': spec['accepted_vals']}, ParamValidator.external_param]

        return param_schema


    def p_array(spec):
        """
        Defines an array parameter.

        :param spec: A dictionary containing the specification of the parameter with specific keys and values:
            - 'name' (str): The name of the specified parameter. Required.
            - 'description' (str): The description of the specified parameter. Required.
            - 'default' (list): The default value of the specified parameter. Must belong to the accepted_vals and belong to the specified type. Required.
            - 'required' (bool): Indicates if the parameter is required. Optional.
            - 'type' (str): String specifying the type of enum. The allowed types are number, integer, string, and boolean. Required.
            - 'minimum' (number): Minimum value allowed. Optional.
            - 'maximum' (number): Maximum value allowed. Optional.
            - 'minItems' (int): Minimum number of items allowed. Optional.
            - 'maxItems' (int): Maximum number of items allowed. Optional.
        :return: A dictionary containing a JSON schema defining the parameter.
        """
        allowed_specs = {"name", "description", "default", "required", "type", "minimum", "maximum", "minItems", "maxItems"}
        ParamValidator.validate_allowed_specs(allowed_specs, spec, "array")

        param_schema = ParamValidator.param_generic(spec)  # pre-fills generic data necessary by all types

        if not spec.get('type'):
            raise ValueError(f"The type for array parameter {spec['name']} was not specified. The valid types for are: number, integer, string, and boolean.")
        
        # if spec['type'] not in ['number', 'integer', 'string', 'boolean']:
        #     raise ValueError(f"Incorrect specified type for array parameter {spec['name']}. The only valid types for array parameters are: number, integer, string, and boolean.")

        match spec['type']:
            case "number":
                type_p = float
            case "integer":
                type_p = int
            case "string":
                type_p = str
            case "boolean":
                type_p = bool
            case _:
                raise ValueError(f"Incorrect specified type for array parameter {spec['name']}. The only valid types for array parameters are: number, integer, string, and boolean.")


        if spec.get('minimum') is not None and spec['type'] not in ['number', 'integer']:
            raise ValueError(f"No minimum value is allowed to be specified since the type of the array {spec['name']} is not number or integer.")

        if spec.get('maximum') is not None and spec['type'] not in ['number', 'integer']:
            raise ValueError(f"No maximum value is allowed to be specified since the type of the array {spec['name']} is not number or integer.")

        tab_fields = {'type': "array", 'items': {'type': spec['type']}}

        if spec.get('minimum') is not None:
            if isinstance(spec['minimum'], (int, float)):
                tab_fields['items']['minimum'] = spec['minimum']
            else:
                raise ValueError(f"The minimum value specified for parameter {spec['name']} should be a number.")

        if spec.get('maximum') is not None:
            if isinstance(spec['maximum'], (int, float)):
                tab_fields['items']['maximum'] = spec['maximum']
            else:
                raise ValueError(f"The maximum value specified for parameter {spec['name']} should be a number.")

        if spec.get('minItems') is not None:
            if isinstance(spec['minItems'], int) and spec['minItems'] > 0:
                tab_fields['minItems'] = spec['minItems']
            else:
                raise ValueError(f"The minItems value specified for parameter {spec['name']} should be a positive integer.")

        if spec.get('maxItems') is not None:
            if isinstance(spec['maxItems'], int) and spec['maxItems'] > 0:
                tab_fields['maxItems'] = spec['maxItems']
            else:
                raise ValueError(f"The maxItems value specified for parameter {spec['name']} should be a positive integer.")

        if not spec.get('default'):
            raise ValueError(f"The default value for array parameter {spec['name']} was not specified.")
        else:
            for value in spec['default']:

                if not isinstance(value, type_p):
                    raise ValueError(f"One of the specified default elements of the array parameter {spec['name']} is not of the same type as the defined type.")

                if spec.get('maximum') and value > spec['maximum']:
                    raise ValueError(f"One of the specified default elements of the array parameter {spec['name']} is above the specified maximum value.")

                if spec.get('minimum') and value < spec['minimum']:
                    raise ValueError(f"One of the specified default elements of the array parameter {spec['name']} is below the specified minimum value.")

            if spec.get('maxItems') and len(spec['default']) > spec['maxItems']:
                raise ValueError(f"The number of elements in the default array {spec['name']} is above the specified maximum value (maxItems).")

            if spec.get('minItems') and len(spec['default']) < spec['minItems']:
                raise ValueError(f"The number of elements in the default array {spec['name']} is below the specified minimum value (minItems).")

        param_schema[spec['name']]['oneOf'] = [tab_fields, ParamValidator.external_param]

        return param_schema



    def parameters(task_description, param_tab):
        """
        Generates a JSON Schema file based on the parameters defined.
        
        :param task_description: (str) A string containing a description of the task specification.
        :param param_tab: (list) A list containing the JSON Schema specification of all individual parameters.
        :return: function_tab (dict) A dictionary containing important functions to load and get the defined parameters.
        """

        # Obtains the filename where this function is called
        filename_lua = os.path.basename(__file__)  # Extract the filename from the full path
        filename_json = filename_lua.replace(".lua", "") + ".json"  # Replace .lua with .json
        filename_no_ext = filename_json.replace(".json", "").replace(".etasl", "")  # Remove extensions

        def load_json_string(json_string):
            """
            Loads a JSON string and returns the parsed JSON object.
            
            :param json_string: (str) A JSON string.
            :return: dict - The parsed JSON object.
            """
            try:
                return json.loads(json_string)
            except json.JSONDecodeError as err:
                raise ValueError(f"Error parsing JSON: {err}")

        def load_json_file(file_path):
            """
            Loads a JSON file and returns the parsed JSON object.
            
            :param file_path: (str) Path to the JSON file.
            :return: dict - The parsed JSON object.
            """
            with open(file_path, 'r') as f:
                json_string = f.read()
            return load_json_string(json_string)

        def write_json_schema(task_description, param_tab):
            """
            Generates a JSON Schema file based on the parameters defined.
            
            :param task_description: (str) A string containing a description of the task specification.
            :param param_tab: (list) A list containing the partial JSON Schema specification of all individual parameters.
            :return: dict - A dictionary containing the full generated JSON Schema.
            """
            task_library_json = {"name": "", "version": "", "description": "", "authors": []}
            file_path_libs = ""

            if "_LUA_FILEPATH_TO_GENERATE_JSON_SCHEMA" in globals():
                task_library_json = load_json_file(os.path.join("_LUA_FILEPATH_TO_GENERATE_JSON_SCHEMA", "../task_library.json"))
                file_path_libs = os.path.dirname("_LUA_FILEPATH_TO_GENERATE_JSON_SCHEMA")
                if not file_path_libs:
                    raise ValueError(f"Invalid file structure for task specification libraries: {_LUA_FILEPATH_TO_GENERATE_JSON_SCHEMA}")

            lib_name = task_library_json["name"]
            lib_version = task_library_json["version"]
            lib_description = task_library_json["description"]

            descript = "Parameters needed to the corresponding task specification in eTaSL"
            if not param_tab:
                descript = "Parameters needed to the corresponding task specification in eTaSL. No parameters specified."

            lib_identifier = f"from-{lib_name}-version-{lib_version}"
            task_identifier = f"is-{filename_no_ext}"

            lib_description = f"Library {lib_name} version {lib_version}.\nAuthors: " + ", ".join(
                [f"{author['name']} ({author['affiliation']})" for author in task_library_json["authors"]]
            )

            def_properties = {
                lib_identifier: {
                    "description": lib_description,
                    "type": "boolean",
                    "const": True
                }
            }

            schema = {
                "$schema": "http://json-schema.org/draft-06/schema#",
                "$id": filename_no_ext,
                "title": "Task Specification Configuration",
                "description": descript,
                "type": "object",
                "properties": def_properties,
                "dependencies": {
                    lib_identifier: {
                        "properties": {
                            task_identifier: {
                                "description": f"Task Specification: {filename_lua}\nSet to true to indicate that the task uses the task specification mentioned above.\nTask description: {task_description}",
                                "type": "boolean",
                                "const": True
                            }
                        },
                        "required": [task_identifier]
                    },
                    task_identifier: {
                        "properties": {},
                        "required": ["file_path", "parameters"]
                    }
                },
                "required": [lib_identifier],
                "additionalProperties": True,
            }

            filepath_lua = f"$/etasl_ros2_application_template/task_specifications/libraries/{file_path_libs}/task_specifications/{filename_lua}"
            schema["dependencies"][task_identifier]["properties"]["file_path"] = {
                "description": "File path of the corresponding task specification", 
                "type": "string", 
                "const": filepath_lua
            }
            schema["dependencies"][task_identifier]["properties"]["parameters"] = {
                "type": "object", 
                "description": "List of parameters needed to define an instance of the task specification", 
                "required": [], 
                "additionalProperties": False, 
                "properties": {}
            }

            for param in param_tab:
                key_name = list(param.keys())[0]
                schema["dependencies"][task_identifier]["properties"]["parameters"]["properties"][key_name] = param[key_name]

            if "_LUA_FILEPATH_TO_GENERATE_JSON_SCHEMA" in globals():
                with open(os.path.join("_LUA_FILEPATH_TO_GENERATE_JSON_SCHEMA", filename_json), "w") as file:
                    json.dump(schema, file, indent=4)
                print("Code exited after generating the JSON Schema file.")
                os._exit(0)

            return schema

        parameter_schema = write_json_schema(task_description, param_tab)

        def get(var_name):
            """
            Gets a parameter that was previously defined with the parameters function and loaded.
            
            :param var_name: (str) The name of the requested parameter.
            :return: The value of the parameter with the corresponding type.
            """
            global _TABLE_CONTAINING_ETASL_PARAMS

            if '_TABLE_CONTAINING_ETASL_PARAMS' not in globals():
                raise ValueError("No parameters have been loaded. Please call one of the load_json methods available.")

            des_var = _TABLE_CONTAINING_ETASL_PARAMS.get(var_name)
            if des_var is not None:
                return des_var
            else:
                is_required = var_name in required_parameters
                if not is_required:
                    default_val = None
                    for param in param_tab:
                        if var_name in param:
                            default_val = param[var_name].get("default")
                    return default_val
                else:
                    raise ValueError(f"Parameter `{var_name}` is required but not found in the JSON file.")

        def load_json_table(json_table):
            """
            Loads a JSON table and validates it according to the schema.
            
            :param json_table: (dict) The JSON table to be validated.
            """
            global _TABLE_CONTAINING_ETASL_PARAMS

            # Remove the "$schema" value
            json_table.pop("$schema", None)

            for key_param, value in json_table.items():
                is_valid = False
                for param in param_tab:
                    if key_param in param:
                        is_valid = True
                        break
                if not is_valid:
                    raise ValueError(f"Parameter {key_param} is not defined in the JSON schema.")
                if isinstance(value, str) and value.startswith("$blackboard"):
                    raise ValueError(f"Parameter {key_param} cannot begin with $blackboard.")

            # Validate the JSON table using the schema
            myvalidator = jsonschema.generate_validator(parameter_schema["parameters"])
            is_valid, error_msg = myvalidator(json_table)

            if not is_valid:
                raise ValueError(f"Validation failed: {error_msg}")

            _TABLE_CONTAINING_ETASL_PARAMS = json_table

        function_tab = {
            "get": get,
            "load_json_table": load_json_table,
            "load_json_string": load_json_string,
            "load_json_file": load_json_file
        }

        return function_tab
