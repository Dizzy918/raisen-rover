#!/usr/bin/env bash
# Re-apply the three local vision fixes to the running narsil-vision container.
#
# These live in the container filesystem, NOT in the narsil source repo, so any
# `docker rm` / watchtower image pull wipes them. That already happened once:
# the container was recreated 2026-08-11 17:24 and all three reverted silently.
# Run this after any container recreate, or land the changes upstream for real.
#
# Usage:  ./apply_to_container.sh [--check]
#   --check  report whether each fix is present, change nothing
#
# Plain parallel arrays, not `declare -A`: macOS ships bash 3.2, which has no
# associative arrays and mis-parses DEST[foo.py] as an arithmetic expression.
set -euo pipefail

HOST=narsil@jetson9.local
CTR=narsil-vision
SHARE=/opt/narsil_ws/install/share/narsil_bringup_vision
HERE="$(cd "$(dirname "$0")" && pwd)"

FILES=(_common.py           ar0234.yaml          camera_csi.yaml)
DESTS=("$SHARE/launch/_common.py" "$SHARE/config/ar0234.yaml" "$SHARE/config/camera_csi.yaml")
MARKS=(ar0234_right_debayer auto_exposure_max    ar0234_fullres)

rc=0
for i in "${!FILES[@]}"; do
  f=${FILES[$i]}; dest=${DESTS[$i]}; mark=${MARKS[$i]}

  if [[ "${1:-}" == "--check" ]]; then
    if ssh "$HOST" "sudo docker exec $CTR grep -q '$mark' '$dest'" 2>/dev/null; then
      printf 'PRESENT  %-16s (%s)\n' "$f" "$mark"
    else
      printf 'MISSING  %-16s (%s)\n' "$f" "$mark"
      rc=1
    fi
    continue
  fi

  echo "==> $f -> $dest"
  # back up the pristine file once, then push the patched copy in
  ssh "$HOST" "sudo docker exec $CTR sh -c '[ -f $dest.orig ] || cp $dest $dest.orig'"
  ssh "$HOST" "sudo docker exec -i $CTR sh -c 'cat > $dest'" < "$HERE/patched/$f"
done

if [[ "${1:-}" == "--check" ]]; then
  exit $rc
fi

echo "==> restarting $CTR"
ssh "$HOST" "sudo docker restart $CTR" >/dev/null
echo "done. verify with: $0 --check"
