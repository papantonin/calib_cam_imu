#!/usr/bin/env bash
# Install build/test dependencies. Source archives remain inside this checkout.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
install_system=1
download_sources=1
optional_packages=()
case "${1:-}" in
  --optical-test-tools) download_sources=0; optional_packages=(python3-opencv) ;;
  --rosbag-tools)
    if (( $# != 1 )); then echo "Too many arguments" >&2; exit 2; fi
    python3 "$project_root/scripts/setup_rosbag_tools.py"
    exit 0 ;;
  --sources-only) install_system=0 ;;
  --system-only) download_sources=0 ;;
  --help|-h)
    echo "Usage: bash scripts/install_dependencies.sh [--system-only|--sources-only|--rosbag-tools|--optical-test-tools]"
    echo "Default: install Ubuntu dependencies, fetch locked sources and build local OpenCV/Ceres."
    exit 0 ;;
  '') ;;
  *) echo "Unknown argument: $1" >&2; exit 2 ;;
esac
if (( $# > 1 )); then echo 'Too many arguments' >&2; exit 2; fi
if (( install_system )); then
  . /etc/os-release
  if [[ "$ID" != ubuntu ]]; then
    echo 'Automatic package installation currently supports Ubuntu only.' >&2
    exit 2
  fi
  privilege=()
  if (( EUID != 0 )); then privilege=(sudo); fi
  "${privilege[@]}" apt-get update
  "${privilege[@]}" apt-get install -y --no-install-recommends \
    g++ cmake ninja-build pkg-config ca-certificates curl python3 python3-matplotlib python3-numpy poppler-utils \
    libcli11-dev libyaml-cpp-dev libpng-dev zlib1g-dev nlohmann-json3-dev libeigen3-dev "${optional_packages[@]}"
fi
if (( download_sources )); then
  python3 "$project_root/scripts/fetch_dependencies.py"
  python3 "$project_root/scripts/setup_opencv.py"
  python3 "$project_root/scripts/setup_ceres.py"
fi
mkdir -p "$project_root/.deps"
dpkg-query -W -f='${binary:Package}\t${Version}\n' \
  g++ cmake ninja-build python3-matplotlib python3-numpy poppler-utils libcli11-dev libyaml-cpp-dev libpng-dev zlib1g-dev nlohmann-json3-dev libeigen3-dev "${optional_packages[@]}" \
  > "$project_root/.deps/system-versions.txt"
echo "Dependency setup complete. See INSTALLATION.md for build and tests."
