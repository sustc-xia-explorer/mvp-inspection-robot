#!/usr/bin/env bash
# Build the copied sources in fresh directories; install without sudo.
set -euo pipefail
mode="${1:-all}"
if [[ "$mode" != core && "$mode" != all ]]; then
  echo 'Usage: bash tools/build_noetic.sh [core|all]' >&2
  exit 2
fi
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -d "$repo/../deps/osqp" ]]; then
  default_deps="$(cd "$repo/../deps" && pwd)"
else
  default_deps="$HOME/work"
fi
deps="${NEUPAN_DEPS_SOURCE_ROOT:-$default_deps}"
build_root="${NEUPAN_BUILD_ROOT:-$HOME/neupan_noetic_build}"
prefix="${NEUPAN_INSTALL_PREFIX:-$HOME/.local/neupan-noetic}"
ros_ws="${NEUPAN_ROS_WS:-$HOME/neupan_noetic_ws}"
cmake_bin="${NEUPAN_CMAKE:-cmake}"
jobs="${NEUPAN_JOBS:-2}"
command -v "$cmake_bin" >/dev/null
cmake_version="$("$cmake_bin" --version | head -n 1)"
python3 - "$cmake_version" <<'PY'
import re, sys
v = tuple(map(int, re.search(r'(\d+)\.(\d+)\.(\d+)', sys.argv[1]).groups()))
if not (3, 18, 0) <= v < (4, 0, 0):
    raise SystemExit('Use CMake 3.18+ within the 3.x series; see NOETIC_SETUP_CN.md.')
PY
for dir in "$deps/osqp" "$deps/osqp-eigen"; do
  [[ -f "$dir/CMakeLists.txt" ]] || { echo "Missing dependency source: $dir" >&2; exit 1; }
done
common=(-DCMAKE_BUILD_TYPE=Release "-DCMAKE_INSTALL_PREFIX=$prefix"
        -DCMAKE_INSTALL_LIBDIR=lib "-DCMAKE_PREFIX_PATH=$prefix"
        "-DCMAKE_INSTALL_RPATH=$prefix/lib")
if [[ -n "${NEUPAN_EIGEN3_DIR:-}" ]]; then
  common+=("-DEigen3_DIR=$NEUPAN_EIGEN3_DIR")
fi
qdldl_args=()
if [[ -f "$deps/qdldl/CMakeLists.txt" ]]; then
  qdldl_args+=("-DFETCHCONTENT_SOURCE_DIR_QDLDL=$deps/qdldl")
elif [[ -f "$deps/osqp/build/_deps/qdldl-src/CMakeLists.txt" ]]; then
  qdldl_args+=("-DFETCHCONTENT_SOURCE_DIR_QDLDL=$deps/osqp/build/_deps/qdldl-src")
fi
mkdir -p "$build_root" "$prefix"
"$cmake_bin" -S "$deps/osqp" -B "$build_root/osqp" "${common[@]}" \
  "${qdldl_args[@]}" -DOSQP_BUILD_UNITTESTS=OFF -DOSQP_BUILD_DEMO_EXE=OFF \
  -DOSQP_BUILD_SHARED_LIB=ON -DOSQP_BUILD_STATIC_LIB=ON
"$cmake_bin" --build "$build_root/osqp" --parallel "$jobs"
"$cmake_bin" --install "$build_root/osqp"
"$cmake_bin" -S "$deps/osqp-eigen" -B "$build_root/osqp-eigen" \
  "${common[@]}" -DBUILD_TESTING=OFF "-Dosqp_DIR=$prefix/lib/cmake/osqp"
"$cmake_bin" --build "$build_root/osqp-eigen" --parallel "$jobs"
"$cmake_bin" --install "$build_root/osqp-eigen"
export LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$cmake_bin" -S "$repo/libneupan" -B "$build_root/libneupan" \
  "${common[@]}" -DNEUPAN_BUILD_TESTS=ON -DNEUPAN_NATIVE=OFF \
  "-DOsqpEigen_DIR=$prefix/lib/cmake/OsqpEigen"
"$cmake_bin" --build "$build_root/libneupan" --parallel "$jobs"
# ctest is taken from the selected CMake distribution when available.
ctest_bin="$(dirname -- "$(command -v "$cmake_bin")")/ctest"
(
  cd "$build_root/libneupan"
  "$ctest_bin" --output-on-failure
)
"$cmake_bin" --install "$build_root/libneupan"
if [[ "$mode" == core ]]; then
  echo "Core build and tests completed: $prefix"
  exit 0
fi
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo 'Core passed. ROS1 build requires /opt/ros/noetic/setup.bash on the IPC.' >&2
  exit 1
fi
# ROS setup files are not nounset-safe.
set +u
source /opt/ros/noetic/setup.bash
if [[ -n "${NEUPAN_DRIVER_WS:-}" ]]; then
  [[ -f "$NEUPAN_DRIVER_WS/devel/setup.bash" ]] || {
    echo "Build the FW-mini driver workspace first: $NEUPAN_DRIVER_WS" >&2; exit 1;
  }
  source "$NEUPAN_DRIVER_WS/devel/setup.bash"
  python3 -c 'from yhs_can_msgs.msg import ctrl_cmd'
fi
set -u
export LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CMAKE_PREFIX_PATH="$prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PATH="$(dirname -- "$(command -v "$cmake_bin")"):$PATH"
mkdir -p "$ros_ws/src"
link="$ros_ws/src/neupan_cpp_ros1"
if [[ -e "$link" || -L "$link" ]]; then
  [[ "$(readlink -f "$link")" == "$repo/neupan_cpp_ros1" ]] || {
    echo "Refusing to overwrite existing ROS package: $link" >&2; exit 1;
  }
else
  ln -s "$repo/neupan_cpp_ros1" "$link"
fi
ros_extra=()
if [[ -n "${NEUPAN_EIGEN3_DIR:-}" ]]; then
  ros_extra+=("-DEigen3_DIR=$NEUPAN_EIGEN3_DIR")
fi
catkin_make -C "$ros_ws" -j"$jobs" -l"$jobs" \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  "-Dlibneupan_DIR=$prefix/lib/cmake/libneupan" \
  "-DCMAKE_PREFIX_PATH=$prefix;${CMAKE_PREFIX_PATH//:/;}" \
  "-DCMAKE_BUILD_RPATH=$prefix/lib" "-DCMAKE_INSTALL_RPATH=$prefix/lib" \
  "${ros_extra[@]}"
# Emit literal, shell-escaped paths so spaces and different usernames are safe.
{
  printf 'source %q\n' /opt/ros/noetic/setup.bash
  if [[ -n "${NEUPAN_DRIVER_WS:-}" ]]; then
    printf 'source %q\n' "$NEUPAN_DRIVER_WS/devel/setup.bash"
  fi
  printf 'source %q\n' "$ros_ws/devel/setup.bash"
  printf 'export LD_LIBRARY_PATH=%q"${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"\n' "$prefix/lib"
} > "$build_root/activate.bash"
echo "ROS1 build completed. Run: source $build_root/activate.bash"
