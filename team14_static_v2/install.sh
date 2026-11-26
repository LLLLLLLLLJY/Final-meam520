#!/usr/bin/env bash
# Install into the course workspace from a USB stick or extracted download.
set -eu
src=$(cd -- "$(dirname -- "$0")" && pwd -P)
lab=$(cd -- "${MEAM520_FINAL_DIR:-$HOME/meam520_ws/src/meam520_labs/labs/final}" && pwd -P)
dst="$lab/team14_static_v2"
case ${1:-} in ''|--competition) ;; *) echo 'Usage: bash install.sh [--competition]' >&2; exit 2;; esac
for f in final.py static_pick.py static_stack.py dynamic_pick.py trial_log.py competition_entry.py kinematics/IK_position_null.py; do
    test -f "$src/$f" || { echo "Missing project file: $f" >&2; exit 1; }
done
stamp=$(date -u +%Y%m%dT%H%M%SZ)-$$
if [ "$src" != "$dst" ]; then
    temp=$(mktemp -d "$lab/.team14-install.XXXXXX")
    cp -R -- "$src/." "$temp/"
    if [ -e "$dst" ]; then
        mv -- "$dst" "$dst.backup.$stamp"
        echo "Previous project: $dst.backup.$stamp"
    fi
    if ! mv -- "$temp" "$dst"; then
        if [ -d "$dst.backup.$stamp" ] && [ ! -e "$dst" ]; then mv -- "$dst.backup.$stamp" "$dst"; fi
        exit 1
    fi
fi
if [ "${1:-}" = --competition ] && ! cmp -s "$dst/competition_entry.py" "$lab/final.py"; then
    if [ -e "$lab/final.py" ]; then cp -p -- "$lab/final.py" "$lab/final.py.backup.$stamp"; fi
    entry=$(mktemp "$lab/.team14-entry.XXXXXX")
    cp -- "$dst/competition_entry.py" "$entry"
    chmod 644 "$entry"
    mv -- "$entry" "$lab/final.py"
fi
echo "Project ready: $dst/final.py"
