#!/usr/bin/env bash
# Revalidate only virtual robots. Does not start a chassis, radar or CAN node.
set -euo pipefail
mode="${1:-core}"
pkg="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
output="${2:-$HOME/neupan_sim_results/omni_ipc_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$output"
output="$(cd -- "$output" && pwd)"
case "$mode" in
  core)
    binary="${NEUPAN_OMNI_CORE_BINARY:-$HOME/neupan_omni_build/core/neupan_ros1_sim_backend}"
    [[ -x "$binary" ]] || { echo "Missing core backend: $binary. Run build_ipc.sh first." >&2; exit 1; }
    ctest_bin="${NEUPAN_OMNI_CTEST:-ctest}"
    (cd -- "$(dirname -- "$binary")" && "$ctest_bin" --output-on-failure) | tee "$output/ctest.log"
    python3 -m unittest discover -s "$pkg/test" -p 'test_*.py' -v 2>&1 | tee "$output/python_tests.log"
    python3 "$pkg/test/model_closed_loop.py" --binary "$binary" \
      --config "$pkg/config/fw_mini_omni.yaml" --output "$output/core_omni"
    python3 "$pkg/test/model_closed_loop.py" --binary "$binary" \
      --config "$pkg/config/fw_mini_balanced.yaml" --output "$output/core_diff"
    ;;
  ros)
    binary="${NEUPAN_OMNI_NODE_BINARY:-${NEUPAN_OMNI_WS:-$HOME/neupan_omni_ws}/devel/lib/neupan_cpp_ros1/neupan_node}"
    [[ -x "$binary" ]] || { echo "Missing ROS1 node: $binary. Run build_ipc.sh all first." >&2; exit 1; }
    python3 "$pkg/test/ros1_model_test.py" --node "$binary" \
      --with-master --rviz-corridor --output "$output/ros1"
    ;;
  corridor)
    binary="${NEUPAN_OMNI_CORE_BINARY:-$HOME/neupan_omni_build/core/neupan_ros1_sim_backend}"
    [[ -x "$binary" ]] || { echo "Missing core backend: $binary" >&2; exit 1; }
    python3 "$pkg/test/tune_omni_corridor.py" --binary "$binary" --suite all --output "$output/corridor_core"
    python3 "$pkg/test/tune_omni_corridor.py" --binary "$binary" --suite sequence --output "$output/sequence_core"
    ;;
  ros-smooth)
    python3 "$pkg/test/ros1_smooth_corridor_test.py" --output "$output/ros1_sequence"
    ;;
  stress)
    binary="${NEUPAN_OMNI_CORE_BINARY:-$HOME/neupan_omni_build/core/neupan_ros1_sim_backend}"
    python3 "$pkg/test/stress_omni_navigation.py" --binary "$binary" --cases all --output "$output/stress_core"
    ;;
  *) echo 'Usage: bash tools/validate_ipc.sh core|ros|corridor|ros-smooth|stress [output_directory]' >&2; exit 2 ;;
esac
echo "Validation completed. Results: $output"
