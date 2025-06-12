import argparse
import os
import json
import jsonref
from pathlib import Path
import requests
from urllib.parse import urlparse

def _strip_jsonref(obj):
    """
    Recursively convert jsonref.JsonRef or JsonRef-like objects into plain dicts/lists.
    """
    if isinstance(obj, jsonref.JsonRef):
        return _strip_jsonref(obj.__subject__)
    elif isinstance(obj, dict):
        return {k: _strip_jsonref(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [_strip_jsonref(v) for v in obj]
    else:
        return obj
    
# def _strip_jsonref(obj, path='root'):
#     """
#     Recursively convert jsonref.JsonRef or JsonRef-like objects into plain dicts/lists.
#     Log any suspicious 'null' values to help diagnose problems.
#     """
#     if isinstance(obj, jsonref.JsonRef):
#         try:
#             return _strip_jsonref(obj.__subject__, path)
#         except Exception as e:
#             print(f"[ERROR] Failed to dereference at {path}: {e}")
#             return None

#     elif isinstance(obj, dict):
#         out = {}
#         for k, v in obj.items():
#             new_path = f"{path}/{k}"
#             result = _strip_jsonref(v, new_path)
#             if result is None and v is not None:
#                 print(f"[WARNING] {new_path} became null during dereferencing")
#             out[k] = result
#         return out

#     elif isinstance(obj, list):
#         out = []
#         for i, v in enumerate(obj):
#             new_path = f"{path}[{i}]"
#             result = _strip_jsonref(v, new_path)
#             if result is None and v is not None:
#                 print(f"[WARNING] {new_path} became null during dereferencing")
#             out.append(result)
#         return out

#     else:
#         return obj


def load_and_dereference_schema(schema_path: str) -> dict:
    """
    Loads and dereferences a JSON schema file, replacing all $ref with their full definitions.
    
    :param schema_path: Path to the root schema file.
    :return: Fully dereferenced JSON schema.
    """
    base_path = Path(schema_path).resolve().parent.as_uri()

    with open(schema_path, 'r') as f:
        schema = json.load(f)

    # Automatically resolves local and remote $ref
    dereferenced = jsonref.JsonRef.replace_refs(
        schema,
        base_uri=base_path + '/',  # important for relative file refs
        loader=_custom_loader
    )
    # print(dereferenced)

    # Convert jsonref.JsonRef to plain dict
    return _strip_jsonref(dereferenced)



def _custom_loader(uri: str):
    """
    A custom loader for jsonref that supports both file:// and https://
    """
    parsed = urlparse(uri)
    
    if parsed.scheme in ('http', 'https'):
        response = requests.get(uri)
        response.raise_for_status()
        return response.json()
    elif parsed.scheme == 'file':
        path = os.path.abspath(os.path.join(parsed.netloc, parsed.path))
        with open(path, 'r') as f:
            return json.load(f)
    # elif parsed.scheme == '':  # Plain path (e.g. /home/user/foo.json or ./foo.json)
    #     path = os.path.abspath(uri)
    #     if not os.path.exists(path):
    #         raise FileNotFoundError(f"File not found: {path}")
    #     with open(path, 'r') as f:
    #         return json.load(f)
    else:
        raise ValueError(f"Unsupported URI scheme in $ref: {uri}")

def main():
    parser = argparse.ArgumentParser(description="Dereference a JSON Schema and output a flat version.")
    parser.add_argument("input_schema", help="Path to the main JSON Schema file")
    parser.add_argument("output_schema", help="Path to output the compiled (dereferenced) schema")

    args = parser.parse_args()

    output_path = Path(args.output_schema)
    if output_path.exists():
        output_path.unlink() #Delete the file if it exists

    try:
        compiled = load_and_dereference_schema(args.input_schema)
        compiled["$id"] = Path(args.output_schema).name

        os.makedirs(os.path.dirname(args.output_schema), exist_ok=True)

        with open(args.output_schema, 'w') as f:
            json.dump(compiled, f, indent=2)

        print(f"Compiled schema saved to: {args.output_schema}")

    except Exception as e:
        # Catch circular reference or other schema-loading issues
        fallback_dict = {
            "Error": f"[ERROR] Could not generate json schema, e.g. due to circular reference detected in file {args.input_schema} or unexistant $ref.",
            "Details": f"[Details] {type(e).__name__}: {e}",
        }

        # fallback_msg = f"[ERROR] Could not generate json schema, e.g. due to circular reference detected in file {args.input_schema} or unexistant $ref. \n [DETAILS] {type(e).__name__}: {e}"
        os.makedirs(os.path.dirname(args.output_schema), exist_ok=True)

        with open(args.output_schema, 'w') as f:
            json.dump(fallback_dict, f, indent=2)

        print(fallback_dict["Error"])
        print(fallback_dict["Details"])
        # print(f"[DETAILS] {type(e).__name__}: {e}")


if __name__ == "__main__":
    main()

# compiled_schema = load_and_dereference_schema('schemas/blackboard-schema.json')
# # print(compiled_schema)




# with open('compiled_schema.json', 'w') as f:
#     json.dump(compiled_schema, f, indent=2)