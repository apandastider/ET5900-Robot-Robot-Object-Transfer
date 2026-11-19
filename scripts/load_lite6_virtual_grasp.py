#!/usr/bin/env python3
import re
import json
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

world = "milestone2_multi_robot_transfer"

r = subprocess.run(
    ["gz", "model", "-m", "lite6", "-p"],
    capture_output=True,
    text=True,
    timeout=30,
    check=True
)

m = re.search(r"Model:\s*\[(\d+)\]", r.stdout)

if not m:
    raise RuntimeError("Lite 6 model ID not found")

arm_id = int(m.group(1))

sdf = (
    Path.home()
    / "mrs_tray_ws/src/mrs_tray_demo/models/pick_block.sdf"
)

root = ET.parse(sdf).getroot()
link = root.find(".//model/link")

if link is None:
    raise RuntimeError("Orange block link not found")

child_link = link.attrib["name"]

xml = (
    "<parent_link>link6</parent_link>"
    "<child_model>pick_block</child_model>"
    f"<child_link>{child_link}</child_link>"
    "<attach_topic>/lite6_virtual_grasp/attach</attach_topic>"
    "<detach_topic>/lite6_virtual_grasp/detach</detach_topic>"
    "<output_topic>/lite6_virtual_grasp/state</output_topic>"
)

req = (
    f"entity {{ id: {arm_id} }} "
    "plugins { "
    'name: "gz::sim::systems::DetachableJoint" '
    'filename: "gz-sim-detachable-joint-system" '
    f"innerxml: {json.dumps(xml)} "
    "}"
)

print("Lite 6 model ID:", arm_id)
print("Orange block link:", child_link)

r = subprocess.run(
    [
        "gz", "service",
        "-s", f"/world/{world}/entity/system/add",
        "--reqtype", "gz.msgs.EntityPlugin_V",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "10000",
        "--req", req
    ],
    capture_output=True,
    text=True,
    timeout=20
)

print("Response:", r.stdout)
print("Errors:", r.stderr)

if r.returncode != 0 or "data: true" not in r.stdout.lower():
    raise RuntimeError("Plugin loading was not confirmed")

print("SUCCESS: DetachableJoint plugin load requested")
