"""
calibrate/read_file.py - JSONL data loader for sensor calibration.

Provides simple functions to read JSONL files with optional schema line.
Designed for small/medium datasets where memory efficiency is not critical.
"""

import json
from typing import Any, Dict, List, Optional, Tuple


def read_jsonl(filepath: str) -> Tuple[Optional[Dict[str, Any]], List[Dict[str, Any]]]:
    """
    Read JSONL file and return (schema, records).

    Args:
        filepath: Path to the JSONL file.

    Returns:
        Tuple of (schema dict or None, list of data records).
        First line with _schema key is treated as schema metadata.
        Subsequent lines are treated as data records.

    Example:
        schema, data = read_jsonl("sensor_data.jsonl")
        print(f"Schema: {schema}")
        print(f"Records: {len(data)}")
    """
    schema: Optional[Dict[str, Any]] = None
    records: List[Dict[str, Any]] = []

    with open(filepath, 'r', encoding='utf-8') as f:
        for line_num, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue

            try:
                obj = json.loads(line)
            except json.JSONDecodeError as e:
                # Log error but continue processing remaining lines
                print(f"JSON decode error at line {line_num}: {e}")
                continue

            # First line should contain _schema key
            if schema is None and '_schema' in obj:
                schema = obj.get('_schema', {})
                continue

            # Data records come after schema line
            records.append(obj)

    return schema, records


def validate_schema(schema: Optional[Dict[str, Any]], required_keys: List[str]) -> bool:
    """
    Validate that schema contains required keys.

    Args:
        schema: Schema dict from read_jsonl.
        required_keys: List of required key names.

    Returns:
        True if all required keys are present and not empty.
    """
    if schema is None:
        return False

    return all(key in schema and schema[key] for key in required_keys)
