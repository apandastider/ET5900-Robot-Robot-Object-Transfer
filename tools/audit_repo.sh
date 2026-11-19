#!/usr/bin/env bash
# Read-only repo completeness and privacy audit.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
missing=0
check_file() {
  if [[ -f "$1" ]]; then echo "OK     $1"; else echo "MISSING $1"; missing=$((missing+1)); fi
}
check_pattern() {
  local pattern="$1" label="$2"
  if find src -type f -name "$pattern" -print -quit 2>/dev/null | grep -q .; then echo "OK     $label"; else echo "MISSING $label"; missing=$((missing+1)); fi
}
echo '=== Required experiment files ==='
check_file 'README.md'
check_file 'tray_env.sh'
check_file 'scripts/tray_transfer_fsm.py'
check_file 'scripts/virtual_grasp_hold.py'
check_file 'config/tray_nav2.yaml'
check_file 'config/tray_localization.yaml'
check_file 'config/demo_poses.yaml'
check_file 'src/mrs_tray_demo/worlds/milestone2_tray_low_station.sdf'
check_pattern 'lite6_low_station_spawn.launch.py' 'Lite 6 spawn launch'
check_pattern 'tray_tb4_spawn.launch.py' 'TurtleBot spawn launch'
check_pattern 'lite6_moveit_tray_only.launch.py' 'MoveIt plan-only launch'
if [[ -f maps/milestone2_map.yaml ]]; then
  echo 'OK     maps/milestone2_map.yaml'
  python3 - <<'PY'
from pathlib import Path
import re
s=Path('maps/milestone2_map.yaml').read_text()
m=re.search(r'^\s*image\s*:\s*(.+?)\s*$', s, re.M)
if m:
    value=m.group(1).strip().strip('"\'')
    image=Path(value)
    if not image.is_absolute(): image=Path('maps')/image
    print(('OK     ' if image.exists() else 'MISSING ')+f'saved map image: {value}')
else:print('WARNING map YAML has no image: line')
PY
else
  echo 'MISSING maps/milestone2_map.yaml'; missing=$((missing+1))
fi

echo; echo '=== Syntax validation ==='
python3 -m py_compile scripts/tray_transfer_fsm.py 2>&1 || missing=$((missing+1))
for f in tools/*.sh; do bash -n "$f" || missing=$((missing+1)); done

echo; echo '=== Large files (manual review) ==='
find . -type f -size +20M -not -path '*/.git/*' -print

echo; echo '=== Potential private or machine-specific content (review filenames) ==='
python3 - <<'PY'
from pathlib import Path
import re
rx=re.compile(rb'(/home/[^\s/]+/|ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|-----BEGIN (?:RSA |OPENSSH )?PRIVATE KEY-----|(?i:password\s*[:=]))')
for base in ['src','scripts','config','maps','generated']:
    for f in sorted(Path(base).rglob('*')):
        if not f.is_file() or f.stat().st_size>2_000_000:continue
        try:
            data=f.read_bytes()
            if rx.search(data):print('REVIEW',f)
        except OSError:pass
PY

echo
echo "Required files missing: $missing"
if ((missing)); then
  echo 'This is not yet a complete source snapshot; run collect_from_workstation.sh and review missing files.'
  exit 2
else
  echo 'Required filenames present. This is not a license/security/correctness guarantee; review before publishing.'
fi
