#!/usr/bin/env bash
# Re-run the coverage mission on the two GoldenEye faces (the full check was interrupted). Appends to results/full_check.log
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="$REPO/results/full_check.log"
for W in goldeneye_dam goldeneye_gorge; do
  echo "##### $W (rerun)" | tee -a "$LOG"
  bash "$REPO/tools/wsl_mission_test.sh" $W demo 1800 0.11 full 2>&1 | tr '\r' '\n' | grep -v Killed | grep -A8 '== report' | tee -a "$LOG"
done
echo "##### done" | tee -a "$LOG"
