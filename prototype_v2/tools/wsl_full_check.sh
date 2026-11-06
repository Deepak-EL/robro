#!/usr/bin/env bash
# Everything for the v2 report in one go: hold tests (measured max 400 g vs full throttle) on the three
# faces, then the coverage mission on all three. Output -> results/full_check.log
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="$REPO/results/full_check.log"
mkdir -p "$REPO/results"
: > "$LOG"
for W in winter_dam goldeneye_dam goldeneye_gorge; do
  for THR in 7 full; do
    echo "##### hold $W throttle $THR" | tee -a "$LOG"
    bash "$REPO/tools/wsl_hold_test.sh" $W $THR 2>&1 | tr '\r' '\n' | grep -v Killed | tail -4 | tee -a "$LOG"
  done
done
bash "$REPO/tools/wsl_all_missions.sh" 2>&1 | tee -a "$LOG"
echo "##### done" | tee -a "$LOG"
