#!/usr/bin/env python3
import sys
import subprocess

def main():
    if len(sys.argv) < 3:
        print("Usage: spawn_entity.py <sdf_file> <name> [x] [y] [z]")
        return 1
    
    sdf_file = sys.argv[1]
    name = sys.argv[2]
    x = sys.argv[3] if len(sys.argv) > 3 else "0.0"
    y = sys.argv[4] if len(sys.argv) > 4 else "0.0"
    z = sys.argv[5] if len(sys.argv) > 5 else "0.0"
    
    pose_str = f"position: {{x: {x}, y: {y}, z: {z}}}"
    req = f'sdf_filename: "{sdf_file}", name: "{name}", pose: {{{pose_str}}}'
    
    cmd = [
        "gz", "service",
        "-s", "/world/empty/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000",
        "--req", req
    ]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)
    
    return result.returncode

if __name__ == "__main__":
    sys.exit(main())
