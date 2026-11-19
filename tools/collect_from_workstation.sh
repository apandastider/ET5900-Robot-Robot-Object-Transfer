#!/usr/bin/env bash
# Copy exact source assets from the working ROS 2 workstation into this repository.
# Does not modify source, build anything, spawn robots, or push commits.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${1:-$HOME/mrs_tray_ws}"
MAP="${MRS_MAP_YAML:-}"
if [[ ! -d "$WS" ]]; then
  echo "ERROR: Working source workspace not found: $WS" >&2
  exit 1
fi
python3 - "$WS" "$REPO" "$MAP" <<'PY'
from pathlib import Path
import sys, shutil, fnmatch, datetime, re
source=Path(sys.argv[1]).expanduser().resolve()
repo=Path(sys.argv[2]).expanduser().resolve()
explicit_map=sys.argv[3]
if repo==source or repo in source.parents or source in repo.parents:
    raise SystemExit('ERROR: Extract this GitHub folder OUTSIDE the active ROS workspace')
if not (source/'tray_env.sh').is_file():
    print('WARNING: tray_env.sh not found; verify this is the correct experiment workspace.')

# Collect source, not colcon outputs. Source code copied from workstation ALWAYS
# takes precedence over older generated snapshots in the downloaded bundle.
FOLDERS = ['src','scripts','config','launch','worlds','models','maps','generated','rviz','resources','urdf','meshes','plugins','include','test']
FILES=['tray_env.sh']
SKIP_DIRS={'.git','build','install','log','__pycache__','.venv','venv','node_modules','.cache','.ros','secrets','.colcon','rosbag2'}
SKIP_FILES={'.env','.DS_Store','.gitignore','authorized_keys','credentials.json'}
SKIP_PATTERNS=('*.pyc','*.pyo','*.swp','*~','*.log','*.db3','*.mcap','*.bag','*.pem','*.key','*.p12','*.pfx','*.pt','*.pth','*.ckpt','*.onnx','id_rsa*','id_ed25519*','core.*')
MAX_SIZE=50*1024*1024
copied=[]
skipped=[]

def copy_path(p,t):
    rel=str(p.relative_to(source))
    if p.is_symlink():
        skipped.append((rel,'symlink (inspect target separately)')); return
    if p.is_dir():
        if p.name in SKIP_DIRS:
            skipped.append((rel,'build/cache/private directory')); return
        t.mkdir(parents=True, exist_ok=True)
        for child in sorted(p.iterdir()): copy_path(child,t/child.name)
        return
    if not p.is_file():return
    if p.name in SKIP_FILES or p.name.startswith('.env') or any(fnmatch.fnmatch(p.name,pattern) for pattern in SKIP_PATTERNS):
        skipped.append((rel,'excluded by safety rule')); return
    if p.stat().st_size > MAX_SIZE:
        skipped.append((rel,'file exceeds 50 MiB')); return
    t.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(p,t)
    copied.append(rel)

for name in FOLDERS + FILES:
    path=source/name
    if path.exists() or path.is_symlink(): copy_path(path,repo/name)

# Optional map located outside main experiment workspace.
map_candidates=([Path(explicit_map).expanduser()] if explicit_map else [])
map_candidates += [
    source/'maps/milestone2_map.yaml',
    Path.home()/'mrs_milestone2_ws/src/mrs_milestone2/maps/milestone2_map.yaml',
    Path.home()/'mrs_student_ws/src/mrs_milestone2/maps/milestone2_map.yaml',
]
map_source=next((p for p in map_candidates if p.is_file()),None)
map_status='No external map found; set MRS_MAP_YAML=/absolute/path/map.yaml and rerun'
if map_source:
    dest_map=repo/'maps'/map_source.name
    dest_map.parent.mkdir(parents=True,exist_ok=True)
    yaml_text=map_source.read_text(encoding='utf-8')
    m=re.search(r'(?m)^\s*image\s*:\s*(\S.+|\S+)\s*$',yaml_text)
    if m:
        raw=m.group(1).strip().strip("'\"")
        src_image=Path(raw)
        if not src_image.is_absolute():src_image=map_source.parent/src_image
        if src_image.is_file() and src_image.stat().st_size <= MAX_SIZE:
            target_image=dest_map.parent/src_image.name
            shutil.copy2(src_image,target_image)
            copied.append('maps/'+src_image.name+' (referenced by saved map)')
            # Ensure map YAML is portable relative to its own folder.
            yaml_text=yaml_text[:m.start(1)] + src_image.name + yaml_text[m.end(1):]
            map_status=f'Copied {map_source.name} plus referenced image {src_image.name}'
        else:
            map_status=f'Copied {map_source.name}; WARNING its image asset was not found: {raw}'
    else:
        map_status=f'Copied {map_source.name}; WARNING no image entry parsed'
    dest_map.write_text(yaml_text,encoding='utf-8')
    if 'maps/'+map_source.name not in copied:copied.append('maps/'+map_source.name)

lines=['# Collected original workspace files','',
    f'Collected: {datetime.date.today().isoformat()}','',
    f'Files copied: {len(copied)}; skipped: {len(skipped)}.',
    'The collector did not modify any original ROS workspace files.',
    '', '## Map copy', '', map_status, '',
    '## Copied', '',
    *(['- `'+v+'`' for v in copied] if copied else ['- None']),
    '', '## Skipped', '',
    *(['- `'+v+'` — '+reason for v,reason in skipped] if skipped else ['- None']),
    '', '## Before publishing', '',
    '- Check that the world, model SDF, launch files, local environment, latest FSM and grasp helper are included.',
    '- Inspect generated URDF/controller configuration and launch files for machine-specific paths.',
    '- Verify map file/image and license of third-party assets.',
    '- Document external xarm/TurtleBot workspace commits and any local patches.',
    '- Inspect the git diff and audit output before pushing.',
]
(repo/'WORKSPACE_INVENTORY.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print(f'Collected {len(copied)} source files; skipped {len(skipped)} entries.')
print(map_status)
print('Inventory:',repo/'WORKSPACE_INVENTORY.md')
print('Next: bash tools/audit_repo.sh')
PY
