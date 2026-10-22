#!/usr/bin/env bash
# Build, validate and install the CLI tools into a relocatable Linux prefix.
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
install_prefix="${HOME:?HOME is required}/.local"
build_dir="$project_root/build/install"
jobs=4
install_dependencies=1
run_tests=1
cxx_standard=23

usage() {
  cat <<'EOF'
Usage: bash scripts/install.sh [options]

Options:
  --prefix DIR          Installation prefix (default: $HOME/.local)
  --jobs N              Parallel build jobs (default: 4)
  --skip-dependencies   Do not run scripts/install_dependencies.sh
  --skip-tests          Build only production targets and skip CTest
  --cxx-standard 23|26  Select the explicit C++ language level (default: 23)
  -h, --help            Show this help

The installer never edits shell startup files. Add PREFIX/bin to PATH yourself
when the selected prefix is not already visible.
EOF
}

while (($#)); do
  case "$1" in
    --prefix)
      (($# >= 2)) || { echo '--prefix requires a directory' >&2; exit 2; }
      install_prefix="$2"; shift 2 ;;
    --jobs)
      (($# >= 2)) || { echo '--jobs requires a positive integer' >&2; exit 2; }
      jobs="$2"; shift 2 ;;
    --skip-dependencies) install_dependencies=0; shift ;;
    --skip-tests) run_tests=0; shift ;;
    --cxx-standard)
      (($# >= 2)) || { echo '--cxx-standard requires 23 or 26' >&2; exit 2; }
      cxx_standard="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$install_prefix" = /* ]] || { echo '--prefix must be an absolute path' >&2; exit 2; }
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo '--jobs must be a positive integer' >&2; exit 2; }
[[ "$cxx_standard" = 23 || "$cxx_standard" = 26 ]] || { echo '--cxx-standard must be 23 or 26' >&2; exit 2; }

if ((install_dependencies)); then
  bash "$project_root/scripts/install_dependencies.sh"
fi

python_executable=/usr/bin/python3
[[ -x "$python_executable" ]] || {
  echo 'System Python is missing; run without --skip-dependencies on Ubuntu.' >&2
  exit 2
}
"$python_executable" -c 'import matplotlib, numpy'
ninja_executable="$(command -v ninja || true)"
if [[ -z "$ninja_executable" && -x "$project_root/.deps/rosbag-venv/bin/ninja" ]]; then
  ninja_executable="$project_root/.deps/rosbag-venv/bin/ninja"
fi
[[ -n "$ninja_executable" ]] || {
  echo 'Ninja is missing; run without --skip-dependencies or install ninja-build.' >&2
  exit 2
}

testing=OFF
if ((run_tests)); then testing=ON; fi
configure_options=()
local_cli11="$project_root/.deps/cli11-local/usr/share/cmake/CLI11"
if [[ -f "$local_cli11/CLI11Config.cmake" ]]; then
  configure_options+=("-DCLI11_DIR=$local_cli11")
fi
cmake -S "$project_root" -B "$build_dir" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_MAKE_PROGRAM="$ninja_executable" \
  -DCMAKE_INSTALL_PREFIX="$install_prefix" \
  -DCALIB_CXX_STANDARD="$cxx_standard" \
  -DPython3_EXECUTABLE="$python_executable" \
  -DBUILD_TESTING="$testing" \
  "${configure_options[@]}"
cmake --build "$build_dir" --parallel "$jobs"
if ((run_tests)); then
  ctest --test-dir "$build_dir" --output-on-failure
fi

permission_root="$install_prefix"
while [[ ! -e "$permission_root" && "$permission_root" != / ]]; do
  permission_root="$(dirname -- "$permission_root")"
done
install_command=(cmake --install "$build_dir" --prefix "$install_prefix" --component Runtime --strip)
if [[ -w "$permission_root" ]]; then
  "${install_command[@]}"
elif ((EUID == 0)); then
  "${install_command[@]}"
else
  sudo "${install_command[@]}"
fi

camera_command="$install_prefix/bin/calib_cameras"
imu_command="$install_prefix/bin/calib_imu_camera"
renderer="$install_prefix/share/cam_imu_calib/render_vi_report.py"
for installed_file in "$camera_command" "$imu_command" "$renderer"; do
  [[ -f "$installed_file" ]] || { echo "Missing installed file: $installed_file" >&2; exit 5; }
done
[[ -x "$camera_command" && -x "$imu_command" ]] || { echo 'Installed commands are not executable' >&2; exit 5; }
"$python_executable" -c 'import pathlib, sys; compile(pathlib.Path(sys.argv[1]).read_text(), sys.argv[1], "exec")' "$renderer"
"$camera_command" --version
"$imu_command" --version

installed_bytes=$(stat -c '%s' "$camera_command" "$imu_command" "$renderer" | awk '{total+=$1} END {print total}')
echo "Installed cam_imu_calib in $install_prefix (${installed_bytes} bytes for commands and report renderer)."
case ":${PATH}:" in
  *":$install_prefix/bin:"*) ;;
  *) echo "Add $install_prefix/bin to PATH to invoke the commands without an absolute path." ;;
esac
