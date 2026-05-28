#!/bin/bash
# Sync one VLM_ROS branch to SysNav, preserving SysNav's git history.
#
# Usage:
#   ~/bin/sync_sysnav.sh <branch>
#   ~/bin/sync_sysnav.sh main && ~/bin/sync_sysnav.sh unitree_go2 && ~/bin/sync_sysnav.sh unitree_g1
#
# Flow per sync:
#   1. Clone SysNav branch (keeps its history)
#   2. Clone VLM_ROS branch + init submodules (source of truth)
#   3. Collect tracked-file list (parent tree + flattened submodules; skip gitlink entries)
#   4. Wipe SysNav working tree (keep .git), rsync VLM_ROS content in
#   5. Commit on top of SysNav's existing history, normal push (no force)
#
# Trade-off: SysNav commits = file-level snapshots, not VLM_ROS's per-commit history.
# The commit message references the VLM_ROS short SHA for traceability.

set -euo pipefail

if [ $# -ne 1 ]; then
    echo "Usage: $0 <branch>" >&2
    exit 1
fi

BRANCH="$1"
SRC="/home/vln/project/codes/VLM_ROS"
WORK=$(mktemp -d)
trap "rm -rf $WORK" EXIT

echo "=== Syncing $BRANCH ==="

# 1. Clone SysNav (keeps its history)
git clone --branch "$BRANCH" --single-branch \
    git@github.com:zwandering/SysNav.git "$WORK/sys" 2>&1 | tail -2

# 2. Clone VLM_ROS + init submodules
git clone --branch "$BRANCH" --single-branch "$SRC" "$WORK/src" 2>&1 | tail -2
cd "$WORK/src"
git submodule update --init --recursive 2>&1 | tail -3
VLMROS_SHA=$(git rev-parse --short HEAD)

# 3. Collect tracked file list (parent + submodules, skip gitlinks and .gitmodules)
TRACKED="$WORK/tracked.txt"
git ls-files --stage \
    | awk '$1!="160000"{print substr($0, index($0,$4))}' \
    | grep -v '^\.gitmodules$' > "$TRACKED"
git submodule foreach --quiet --recursive \
    'git ls-files | sed "s|^|$displaypath/|"' >> "$TRACKED"
echo "Tracked files: $(wc -l < "$TRACKED")"

# 4. Wipe SysNav working tree (keep .git), rsync new content in
cd "$WORK/sys"
git ls-files -z | xargs -0 -r rm -f
rsync -a --files-from="$TRACKED" "$WORK/src/" "$WORK/sys/"

# 5. Commit on top of SysNav's history
cd "$WORK/sys"
git add -f -A
if git diff --cached --quiet; then
    echo "$BRANCH: already in sync with VLM_ROS @ $VLMROS_SHA"
else
    git -c user.email="haokunz@andrew.cmu.edu" -c user.name="Haokun Zhu" \
        commit -q -m "Sync from VLM_ROS $BRANCH @ $VLMROS_SHA"
    git push origin "$BRANCH" 2>&1 | tail -3
    echo "$BRANCH: synced"
fi
