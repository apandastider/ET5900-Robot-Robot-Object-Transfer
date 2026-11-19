# Publish this demonstration to GitHub

Target: **https://github.com/apandastider/ET5900-Robot-Robot-Object-Transfer**

This package is a **repository-preparation bundle**, not evidence that anything has been uploaded already. GitHub authentication is required to push. The commands below operate from the Ubuntu workstation holding the working `~/mrs_tray_ws` source.

## 1. Download and extract the prepared source

Copy the ZIP file onto the workstation, then:

```bash
cd ~/Downloads
unzip ET5900-Robot-Robot-Object-Transfer-github-ready.zip
cd ET5900-Robot-Robot-Object-Transfer
```

## 2. Collect your *actual* running ROS 2 source

```bash
bash tools/collect_from_workstation.sh
bash tools/audit_repo.sh
```

This copies package source (`src/`), launch files, SDF models/worlds, Python scripts including the most recent **locally tested** FSM, Nav2/AMCL settings, maps, generated robot settings, and `tray_env.sh`. It does **not** copy `build/`, `install/`, `log/` or installed third-party binaries.

Verify the inventory (`WORKSPACE_INVENTORY.md`). Make sure `scripts/virtual_grasp_hold.py`, `src/mrs_tray_demo/worlds/milestone2_tray_low_station.sdf`, the live launch files, the orange-block SDF, and the saved map/image exist. The collector *overwrites bundled files of the same name with current workstation files* intentionally, so the actual tested FSM takes precedence over any archived copy.

Inspect before publishing:

```bash
find src scripts config maps generated -type f | sort
rg -n '/home/|password|token|api_key|secret|private_key' \
  src scripts config maps generated tray_env.sh || true
```

Remove or rewrite local absolute paths and credentials, and make sure any upstream assets can be redistributed. Do not publish proprietary material or secrets.

## 3. Copy into the existing GitHub repository and push

Do not `git init` in the original ROS workspace. Instead clone the target into a **separate** directory. This preserves any README/commit history already on GitHub:

```bash
cd ~/Downloads
# If the GitHub repo already exists and is accessible:
git clone https://github.com/apandastider/ET5900-Robot-Robot-Object-Transfer.git ET5900-GitHub-Publish

# Copy the prepared source into the clone, leaving its .git untouched:
rsync -av --exclude='.git/' \
  ~/Downloads/ET5900-Robot-Robot-Object-Transfer/ \
  ~/Downloads/ET5900-GitHub-Publish/

cd ~/Downloads/ET5900-GitHub-Publish
git status --short
git diff --check
git add .
git diff --cached --stat
git commit -m 'Add ET5900 multi-robot transfer simulation and FSM'
git push -u origin HEAD
```

If `git clone` fails with 404, first ensure the repository exists under the exact owner/name and that your GitHub credentials have access. If it fails due to authentication, sign in via GitHub CLI (`gh auth login`) or your configured SSH credentials. Do not use passwords as Git HTTPS credentials. If the remote's default branch isn't `main`, `git push -u origin HEAD` pushes the current checked-out branch without overwriting other branches.

If the remote is **not yet created**, create the empty repository at GitHub under `apandastider` using the name `ET5900-Robot-Robot-Object-Transfer`, then repeat the clone procedure. No force pushes are required.

## 4. Update after future experiments

```bash
cd ~/Downloads/ET5900-Robot-Robot-Object-Transfer
bash tools/collect_from_workstation.sh
bash tools/audit_repo.sh
rsync -av --exclude='.git/' ./ ~/Downloads/ET5900-GitHub-Publish/
cd ~/Downloads/ET5900-GitHub-Publish
git status --short
git add .
git commit -m 'Update ET5900 transfer demo'
git push
```

The collector does not automatically delete stale files: review removed/renamed source files manually before committing. Do not commit session logs, build/install directories or unnecessarily large recordings.
