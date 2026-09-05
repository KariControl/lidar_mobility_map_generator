#!/usr/bin/env python3
"""Parse acceptance lock files without executing them as shell code."""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys


IMAGE_PATTERN = re.compile(r"^[A-Za-z0-9._:/-]+@sha256:[0-9a-f]{64}$")
ALLOWED_KEYS = {
    "LMMG_ROS_JAZZY_BASE_IMAGE",
    "LMMG_AUTOWARE_ACCEPTANCE_IMAGE",
    "LMMG_NAV2_ACCEPTANCE_IMAGE",
}


class LockError(ValueError):
    pass


def read_lock(path: pathlib.Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise LockError(f"{path}:{line_number}: expected KEY=VALUE")
        key, value = line.split("=", 1)
        if key not in ALLOWED_KEYS:
            raise LockError(f"{path}:{line_number}: unsupported key {key!r}")
        if key in result:
            raise LockError(f"{path}:{line_number}: duplicate key {key!r}")
        if not IMAGE_PATTERN.fullmatch(value):
            raise LockError(f"{path}:{line_number}: {key} is not digest-qualified")
        if value.endswith("0" * 64):
            raise LockError(f"{path}:{line_number}: all-zero digest is not a lock")
        result[key] = value
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("lock_file", type=pathlib.Path)
    parser.add_argument(
        "--mode", choices=("runtime", "autoware", "nav2", "base"), default="runtime"
    )
    parser.add_argument("--get", choices=sorted(ALLOWED_KEYS))
    args = parser.parse_args()
    try:
        values = read_lock(args.lock_file)
        required_by_mode = {
            "runtime": {
                "LMMG_AUTOWARE_ACCEPTANCE_IMAGE",
                "LMMG_NAV2_ACCEPTANCE_IMAGE",
            },
            "autoware": {"LMMG_AUTOWARE_ACCEPTANCE_IMAGE"},
            "nav2": {"LMMG_NAV2_ACCEPTANCE_IMAGE"},
            "base": {"LMMG_ROS_JAZZY_BASE_IMAGE"},
        }
        required = required_by_mode[args.mode]
        missing = sorted(required - values.keys())
        if missing:
            raise LockError("lock is missing: " + ", ".join(missing))
        if args.get:
            if args.get not in values:
                raise LockError(f"lock has no value for {args.get}")
            print(values[args.get])
        else:
            print(json.dumps(values, sort_keys=True))
    except (OSError, LockError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
