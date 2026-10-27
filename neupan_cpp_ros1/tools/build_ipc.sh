#!/usr/bin/env bash
# Build this package's omni-capable core in a new workspace, without sudo.
set -euo pipefail
mode="${1:-all}"
if [[ "$mode" != core && "$mode" != all ]]; then
  echo 'Usage: bash tools/build_ipc.sh [all|core]' >&2
  exit 2
fi
pkg="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_root="${NEUPAN_OMNI_BUILD_ROOT:-$HOME/neupan_omni_build}"
ros_ws="${NEUPAN_OMNI_WS:-$HOME/neupan_omni_ws}"
ros_setup="${NEUPAN_ROS1_SETUP:-/opt/ros/noetic/setup.bash}"
jobs="${NEUPAN_JOBS:-2}"
if [[ -n "${NEUPAN_CMAKE:-}" ]]; then
  cmake_bin="$NEUPAN_CMAKE"
elif [[ -x /opt/cmake-3.22.6/bin/cmake ]]; then
  cmake_bin=/opt/cmake-3.22.6/bin/cmake
else
  cmake_bin=cmake
fi
cmake_bin="$(command -v "$cmake_bin")"
ctest_bin="$(dirname -- "$cmake_bin")/ctest"
if [[ ! -x "$ctest_bin" ]]; then ctest_bin="$(command -v ctest)"; fi
mkdir -p "$build_root"
build_root="$(cd -- "$build_root" && pwd)"
# Prefer the previously installed Noetic dependencies. Missing dependencies are
# built into the new build directory, never into the old installation prefix.
if [[ -n "${NEUPAN_INSTALL_PREFIX:-}" ]]; then
  prefix="$NEUPAN_INSTALL_PREFIX"
elif [[ -f "$HOME/.local/neupan-noetic/lib/cmake/OsqpEigen/OsqpEigenConfig.cmake" &&
        -f "$HOME/.local/neupan-noetic/lib/cmake/osqp/osqp-config.cmake" ]]; then
  prefix="$HOME/.local/neupan-noetic"
elif [[ -f /usr/local/lib/cmake/OsqpEigen/OsqpEigenConfig.cmake &&
        -f /usr/local/lib/cmake/osqp/osqp-config.cmake ]]; then
  prefix=/usr/local
else
  prefix="$build_root/deps_install"
fi
if [[ ! -f "$prefix/lib/cmake/OsqpEigen/OsqpEigenConfig.cmake" ||
      ! -f "$prefix/lib/cmake/osqp/osqp-config.cmake" ]]; then
  deps="${NEUPAN_DEPS_SOURCE_ROOT:-$pkg/../deps}"
  for name in osqp osqp-eigen qdldl; do
    [[ -f "$deps/$name/CMakeLists.txt" ]] || {
      echo "Missing bundled dependency source: $deps/$name" >&2; exit 1;
    }
  done
  python3 - "$("$cmake_bin" --version)" <<'PY'
import re, sys
v=tuple(map(int,re.search(r'(\d+)\.(\d+)\.(\d+)',sys.argv[1]).groups()))
if not (3,18,0) <= v < (4,0,0):
    raise SystemExit('Building bundled OSQP requires CMake 3.18+ in the 3.x series. Set NEUPAN_CMAKE=/opt/cmake-3.22.6/bin/cmake.')
PY
  mkdir -p "$prefix"
  common=(-DCMAKE_BUILD_TYPE=Release "-DCMAKE_INSTALL_PREFIX=$prefix"
    -DCMAKE_INSTALL_LIBDIR=lib "-DCMAKE_PREFIX_PATH=$prefix"
    "-DCMAKE_INSTALL_RPATH=$prefix/lib")
  "$cmake_bin" -S "$deps/osqp" -B "$build_root/osqp" "${common[@]}" \
    "-DFETCHCONTENT_SOURCE_DIR_QDLDL=$(cd -- "$deps/qdldl" && pwd)" \
    -DOSQP_BUILD_UNITTESTS=OFF -DOSQP_BUILD_DEMO_EXE=OFF \
    -DOSQP_BUILD_SHARED_LIB=ON -DOSQP_BUILD_STATIC_LIB=ON
  "$cmake_bin" --build "$build_root/osqp" --parallel "$jobs"
  "$cmake_bin" --install "$build_root/osqp"
  "$cmake_bin" -S "$deps/osqp-eigen" -B "$build_root/osqp-eigen" \
    "${common[@]}" -DBUILD_TESTING=OFF "-Dosqp_DIR=$prefix/lib/cmake/osqp"
  "$cmake_bin" --build "$build_root/osqp-eigen" --parallel "$jobs"
  "$cmake_bin" --install "$build_root/osqp-eigen"
fi
export LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
eigen_args=()
if [[ -n "${NEUPAN_EIGEN3_DIR:-}" ]]; then
  eigen_args+=("-DEigen3_DIR=$NEUPAN_EIGEN3_DIR")
fi
"$cmake_bin" -S "$pkg/core" -B "$build_root/core" \
  -DCMAKE_BUILD_TYPE=Release -DNEUPAN_ROS1_CORE_TESTS=ON \
  "-DCMAKE_PREFIX_PATH=$prefix" "-DCMAKE_BUILD_RPATH=$prefix/lib" "${eigen_args[@]}"
"$cmake_bin" --build "$build_root/core" --parallel "$jobs"
(cd -- "$build_root/core" && "$ctest_bin" --output-on-failure)
{
  printf 'export NEUPAN_OMNI_PACKAGE_ROOT=%q\n' "$pkg"
  printf 'export NEUPAN_OMNI_BUILD_ROOT=%q\n' "$build_root"
  printf 'export NEUPAN_OMNI_CORE_BINARY=%q\n' "$build_root/core/neupan_ros1_sim_backend"
  printf 'export NEUPAN_OMNI_WS=%q\n' "$ros_ws"
  printf 'export NEUPAN_OMNI_CTEST=%q\n' "$ctest_bin"
  printf 'export LD_LIBRARY_PATH=%q"${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"\n' "$prefix/lib"
} > "$build_root/core_env.bash"
if [[ "$mode" == core ]]; then
  echo "Core build/tests passed. Run: source $build_root/core_env.bash"
  exit 0
fi
[[ -f "$ros_setup" ]] || {
  echo "Core passed; ROS1 setup is missing: $ros_setup" >&2; exit 1;
}
set +u
source "$ros_setup"
set -u
if [[ "${ROS_VERSION:-}" != 1 ]]; then
  echo 'ROS1 is required. Open a ROS Noetic terminal and rerun.' >&2
  exit 1
fi
export LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CMAKE_PREFIX_PATH="$prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PATH="$(dirname -- "$cmake_bin"):$PATH"
mkdir -p "$ros_ws/src"
link="$ros_ws/src/neupan_cpp_ros1"
if [[ -e "$link" || -L "$link" ]]; then
  [[ "$(readlink -f -- "$link")" == "$pkg" ]] || {
    echo "Existing package points to another version: $link" >&2
    echo 'Choose a new NEUPAN_OMNI_WS; no old workspace has been overwritten.' >&2
    exit 1
  }
else
  ln -s -- "$pkg" "$link"
fi
catkin_make -C "$ros_ws" -j"$jobs" -l"$jobs" \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF \
  -DNEUPAN_ROS1_CORE_TESTS=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  "-DCMAKE_PREFIX_PATH=$prefix;${CMAKE_PREFIX_PATH//:/;}" \
  "-DCMAKE_BUILD_RPATH=$prefix/lib" "${eigen_args[@]}"
{
  printf 'source %q\n' "$ros_setup"
  printf 'source %q\n' "$ros_ws/devel/setup.bash"
  printf 'source %q\n' "$build_root/core_env.bash"
  printf 'export ROS_PACKAGE_PATH=%q"${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"\n' "$ros_ws/src"
  printf 'export NEUPAN_OMNI_NODE_BINARY=%q\n' "$ros_ws/devel/lib/neupan_cpp_ros1/neupan_node"
} > "$build_root/activate.bash"
echo "Omni/diff ROS1 build completed. Run: source $build_root/activate.bash"
