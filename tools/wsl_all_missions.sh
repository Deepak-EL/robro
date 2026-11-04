#!/usr/bin/env bash
# Final check: the coverage mission on all three dam faces in a row (full throttle), reports in results/.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for W in winter_dam goldeneye_dam goldeneye_gorge; do
  echo "##### $W"
  bash "$REPO/tools/wsl_mission_test.sh" $W demo 1800 0.11 full 2>&1 | tr '\r' '\n' | grep -v Killed | grep -A8 '== report'
done
