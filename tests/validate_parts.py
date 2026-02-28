
import json
import jsonschema
from jsonschema import validate

def validate_parts_library(schema_path, db_path):
    print(f"Loading schema from {schema_path}...")
    with open(schema_path, 'r', encoding='utf-8') as f:
        schema = json.load(f)

    print(f"Loading database from {db_path}...")
    with open(db_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    print("Validating...")
    try:
        validate(instance=data, schema=schema)
        print("✅ Validation Successful! The parts database conforms to Schema v2.")
    except jsonschema.exceptions.ValidationError as err:
        print(f"❌ Validation Failed!")
        print(f"Message: {err.message}")
        print(f"Path: {list(err.path)}")
        print(f"Instance: {err.instance}")

if __name__ == "__main__":
    validate_parts_library('parts_library/parts_schema_v2.json', 'parts_library/complete_parts_database.json')
