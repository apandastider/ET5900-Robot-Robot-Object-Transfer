#!/usr/bin/env python3
"""Fix duplicate rclpy.shutdown() on SIGINT in an already-working helper."""
from pathlib import Path
p = Path.home() / 'mrs_tray_ws/scripts/virtual_grasp_hold.py'
s = p.read_text()
old = "        rclpy.shutdown()"
new = "        if rclpy.ok():\n            rclpy.shutdown()"
if new in s:
    print('Shutdown guard is already present. No change needed.')
elif old in s:
    backup = p.with_suffix('.py.pre_shutdown_fix.bak')
    backup.write_text(s)
    p.write_text(s.replace(old, new, 1))
    print(f'Patched {p}; backup: {backup}')
else:
    raise SystemExit('Original shutdown line was not found; inspect the helper manually.')
