import argparse
import json
import subprocess
import time
from datetime import datetime

from ntcore import NetworkTableInstance

from robot import Robot
from utils.property import registry


def clear():
    """
    Clear real robot's NetworkTables of persistent properties that no longer exist.

    It is dangerous to run this in Robot.robotInit(): if another branch's code is ru2n on the robot where
    new autoproperties do not exist yet, they will be deleted and set values will be lost.
    """
    inst = NetworkTableInstance.getDefault()
    inst.stopLocal()
    inst.startClient4("clear")
    inst.setServerTeam(5528)
    inst.startDSClient()

    robot = Robot()
    robot.robotInit()

    topics = NetworkTableInstance.getDefault().getTopics()
    registry_keys = list(map(lambda x: x.key, registry))
    for topic in topics:
        name = topic.getName()
        if name.startswith("/Properties/"):
            if name not in registry_keys:
                topic.setPersistent(False)
                print("Deleted unused persistent property:", name)


def save_loop():
    while True:
        save_once()
        time.sleep(30.0)


def save_once():
    print(f"[{datetime.now().time().replace(microsecond=0).isoformat()}] Connecting to robot...")
    proc = subprocess.run(
        "scp -o StrictHostKeyChecking=no -o ConnectTimeout=3 lvuser@10.55.28.2:/home/lvuser/networktables.ini robot_networktables.json"
    )

    # Error code
    if proc.returncode != 0:
        return

    print("Saved properties to robot_networktables.json")

    update_files()


def update_files():
    with open("robot_networktables.json", "r") as f:
        data = json.load(f)

    for entry in data:
        matched_prop = next((prop for prop in registry if prop.key == entry["name"]), None)

        if matched_prop:
            print("Updating", entry["name"])

            with open(matched_prop.filename, "r") as f:
                lines = f.readlines()

            line = lines[matched_prop.line_no]

            # Replace characters after (
            idx_start = line.index("(", matched_prop.col_offset) + 1

            # Replace before ) or ,
            idx_end = line.index(")", matched_prop.col_offset)
            try:
                idx_end = line.index(",", matched_prop.col_offset)
            except ValueError:
                pass

            # Replace old value by new
            line = line[:idx_start] + str(entry["value"]) + line[idx_end:]

            # Replace line
            lines[matched_prop.line_no] = line

            # Rewrite file
            with open(matched_prop.filename, "w") as f:
                f.writelines(lines)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(required=True, help="Enter the command to be executed.")

    # Clean
    parser_clear = subparsers.add_parser(
        "clear",
        help="Clear real robot's NetworkTables of persistent properties that no longer exist."
    )
    parser_clear.set_defaults(func=clear)

    # Save once
    parser_save_once = subparsers.add_parser(
        "saveonce",
        help="Save once real robot's NetworkTables properties to local file."
    )
    parser_save_once.set_defaults(func=save_once)

    # Save loop
    parser_save_loop = subparsers.add_parser(
        "saveloop",
        help="Save periodically real robot's NetworkTables properties to local file."
    )
    parser_save_loop.set_defaults(func=save_loop)

    # Update files
    parser_update_files = subparsers.add_parser(
        "updatefiles",
        help="Update files autoproperties values with robot_networktables.json values."
    )
    parser_update_files.set_defaults(func=update_files)

    args = parser.parse_args()
    args.func()
