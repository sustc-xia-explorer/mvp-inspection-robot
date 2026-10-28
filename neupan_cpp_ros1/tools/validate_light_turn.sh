#!/usr/bin/env bash
# C++ closed-loop validation of the frozen FW-OMNI-LIGHT-TURN-V2 profile.
set -euo pipefail
mode="${1:-core}"
pkg="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "$mode" in
  core) cases=all ;;
  acute) cases=short_acute_zigzag ;;
  *) echo 'Usage: bash tools/validate_light_turn.sh core|acute [new_output_directory]' >&2; exit 2 ;;
esac
output="${2:-$HOME/neupan_sim_results/light_turn_v2_${mode}_$(date +%Y%m%d_%H%M%S)}"
binary="${NEUPAN_OMNI_CORE_BINARY:-$HOME/neupan_omni_light_turn_build/core/neupan_ros1_sim_backend}"
[[ -x "$binary" ]] || { echo "Missing core backend: $binary. Source the new activate.bash first." >&2; exit 1; }
[[ ! -e "$output" ]] || { echo "Output already exists; choose a new directory: $output" >&2; exit 1; }
mkdir -p "$output"
output="$(cd -- "$output" && pwd)"
python3 "$pkg/test/stress_omni_navigation.py" --binary "$binary" \
  --planner "$pkg/config/profiles/ros1/fw_mini_omni_light_turn_v2.yaml" \
  --cases "$cases" --output "$output/core" | tee "$output/run.log"
python3 - "$output" "$mode" <<'PY'
from pathlib import Path
import json,sys
out=Path(sys.argv[1]);mode=sys.argv[2]
rows=json.loads((out/'core/summary.json').read_text())
feasible=[r for r in rows if not r.get('geometric_no_path')]
refusals=[r['case'] for r in rows if r.get('geometric_no_path')]
failures=[{'case':r['case'],'original_pass':r.get('pass',False),
           'max_stall_seconds':r.get('max_stall_seconds')}
          for r in feasible if not r.get('pass') or r.get('max_stall_seconds',float('inf'))>1.5+1e-9]
expected=['random_6619','random_10427','random_14891'] if mode=='core' else []
passed=bool(len(feasible)==(19 if mode=='core' else 1) and sorted(refusals)==sorted(expected) and not failures)
result=dict(profile='FW-OMNI-LIGHT-TURN-V2',passed=passed,total_maps=len(rows),
            feasible=len(feasible),arrived=sum(bool(r.get('arrived')) for r in feasible),
            target_pass=sum(bool(r.get('pass')) and r.get('max_stall_seconds',float('inf'))<=1.5+1e-9 for r in feasible),
            low_speed_threshold_seconds=1.5,
            max_continuous_low_speed_seconds=max((r.get('max_stall_seconds',float('inf')) for r in feasible),default=None),
            no_route=refusals,failures=failures)
(out/'acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
raise SystemExit(0 if passed else 1)
PY
