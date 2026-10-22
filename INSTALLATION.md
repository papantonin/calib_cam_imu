# Installation and build

Initial platform: Ubuntu 26.04 x86_64. Older Ubuntu versions are not yet qualified
and may require a recent compiler/CMake. The script installs dependencies actually
used by the current code and will be extended alongside each new module.

## Usable installation

From the repository root:

```sh
bash scripts/install.sh
```

By default, this command installs dependencies, configures a C++23 release build
in `build/install`, builds with four jobs, runs CTest, then installs the commands
under `~/.local/bin`. The Matplotlib report renderer and target examples go under
`~/.local/share/cam_imu_calib`. The installation contains neither AprilTag
development headers nor its static development library, and it does not modify
shell startup files.

Main options:

```sh
bash scripts/install.sh --prefix /opt/cam_imu_calib --jobs 8
bash scripts/install.sh --skip-dependencies --skip-tests
bash scripts/install.sh --cxx-standard 26
```

The prefix must be absolute. Use `--skip-dependencies` when dependencies have
already been installed and verified; `--skip-tests` is for fast rebuilds, not a
qualified delivery. Manually add `PREFIX/bin` to `PATH` if absent. After installation,
the script prints both executable versions and the size of the binaries plus the
report renderer.

Run installation in an interactive terminal: `sudo` will request your password
when needed, with hidden input. The password is neither passed as an argument nor
stored by the script. There is no password to send to the assistant.

The scripts use sudo for system packages and, only when the selected prefix is not
writable, for the final installation step. They download locked official AprilTag,
OpenCV, and Ceres sources into `.deps/`, verify SHA-256 hashes, then build OpenCV
and Ceres locally with four jobs. Package versions are recorded in
`.deps/system-versions.txt`. AprilTag and these libraries are not globally installed,
and there is no ROS dependency. The application C++ build downloads nothing.
Optional extraction tools use a separate local Python environment described below.

## Uninstalled development build

```sh
bash scripts/install_dependencies.sh
cmake --preset release
cmake --build --preset release
ctest --preset release
```

Options: `--system-only` for apt only; `--sources-only` when packages are already
installed (verified download and local OpenCV/Ceres builds). An interrupted download
never replaces a verified archive. The first OpenCV build takes several minutes;
subsequent runs reuse its objects. To control concurrency after downloading:
`python3 scripts/setup_opencv.py --jobs 4`. The same setting applies to Ceres:
`python3 scripts/setup_ceres.py --jobs 4`. Finish installation before configuring
the project: provenance markers are removed during rebuild and published only after
successful installation.

## Current dependencies

| Dependency | Installation | Role |
| --- | --- | --- |
| GCC/G++, CMake, Ninja | Ubuntu | C++26 build, explicit C++23 fallback |
| CLI11 | `libcli11-dev` | Strict CLI arguments |
| yaml-cpp | `libyaml-cpp-dev` | Configuration and camera calibration |
| libpng, zlib | `libpng-dev`, `zlib1g-dev` | PNG reading and header CRC validation |
| nlohmann JSON | `nlohmann-json3-dev` | Structured results |
| Python 3, Matplotlib, NumPy | Ubuntu | Multipage VI PDF with headless `Agg`, fixtures, and CLI tests |
| AprilTag 3.4.5 | Locked official source | Tag detection |
| OpenCV 5.0.0 | Locked official source, local build | Intrinsic and target-pose initialization |
| Ceres 2.2.0 | Locked official source, local build | Camera–camera and GS camera–IMU optimization |
| Eigen 3.4 | `libeigen3-dev` | Algebra and Ceres sparse solver |
| Poppler | `poppler-utils` | Independent PDF reading in tests |

Ceres 2.2.0 (BSD-3-Clause) is locked to commit
`85331393dc0dff09f6fb9903ab0c4bfa3e134b01`, with SHA-256 in `dependencies.json`.
It uses Eigen Sparse (`SPARSE_SCHUR` for the rig, `SPARSE_NORMAL_CHOLESKY` for the
camera–IMU solver) and Ceres's bundled minimal logging; SuiteSparse, LAPACK, CUDA,
and gflags are disabled. Do not add glog alongside this minimal build. Eigen
3.4.0-5 is the tested system version, selected for compatibility with this Ceres
release without claiming to be the latest upstream Eigen. Apt versions are not
source-locked.

Ceres and its consumers now share `EIGEN_MALLOC_ALREADY_ALIGNED=0`: Eigen explicitly
manages aligned allocations in both configurations. Without this common convention,
ASan changes Eigen autodetection and may mix two allocation ABIs between release
Ceres and the instrumented solver (an invalid free was observed during VI validation).
After updating, rebuild Ceres with `python3 scripts/setup_ceres.py --jobs 2`;
CMake rejects old provenance. Do not bypass this check by removing ASan diagnostics.

Basalt is not integrated. The camera–IMU report is rendered directly by
`scripts/render_vi_report.py` using Matplotlib and `PdfPages` with the `Agg` backend;
the C++ binary links neither Python nor NumPy. CMake verifies imports with the
interpreter the CLI will invoke. Temporary series are removed only after validation
and atomic PDF publication. The camera–camera report remains written in C++.
Poppler independently reads PDFs in tests and is not called by calibration commands.

Stable OpenCV 5.0.0 (Apache-2.0) is locked to commit
`40738fb16ceddb5fb3fea747585f7ce6abb0605b`, with SHA-256 in `dependencies.json`.
The build selects the `calib` module and its transitive dependencies; GUI, video,
DNN, Python, IPP, OpenCL, TBB, OpenMP, and LAPACK are disabled. Neither `fast-math`
nor `-march=native` is used. The backend runs one thread per camera; the application
parallelizes cameras within its global budget. CMake requires the local build and
its provenance; system OpenCV 4.10 is not silently substituted. OpenCV source and
its LICENSE remain under `.deps/`.

### Independent optical oracle (optional)

The C++ optical models add no binary dependency. Their finite-difference tests are
part of the standard build. To also enable independent comparison with Python OpenCV:

```sh
bash scripts/install_dependencies.sh --optical-test-tools
cmake --preset release -DCALIB_OPENCV_TESTS=ON
cmake --build --preset release
ctest --preset release
```

This option installs Ubuntu packages `python3-opencv` and `python3-numpy` in
addition to normal system build dependencies. Their versions are recorded in
`.deps/system-versions.txt` without an apt snapshot lock. OpenCV 4.10.0 served for
initial validation; it is not the production solver. CMake fails explicitly when
the option is enabled and its Python interpreter cannot import `cv2` and `numpy`.
Use system Python (`-DPython3_EXECUTABLE=/usr/bin/python3`) if a virtual environment
hides those packages. The same option is available with the `sanitize` preset.
See [conventions and evidence](docs/camera-models.md).

## AprilTag: release, branch, and Ubuntu package

Verification on October 15, 2026: the
[latest official release](https://github.com/AprilRobotics/apriltag/releases/tag/v3.4.5)
is v3.4.5, commit `94be783968e5091bcc9972c72c84fd63efce2935`. The proposed Ubuntu
package is `3.4.5-4`: the same upstream version with a different packaging revision.
This does not mean its content is byte-for-byte identical to GitHub.

The development branch checked with `git ls-remote` points to
`b7c0ebe9aa20f82ec7a828579004f9e706bfecd9`, which differs from the release. We use
the official release rather than a floating branch. URL, commit, and SHA-256 are
in `dependencies.json`, shared by download and build.

For an update, compare official releases, change the lock with the corresponding
archive hash, then run detection tests and regressions. Do not automatically track
master on every installation.

System libraries are resolved by apt and their versions recorded, not frozen by
an Ubuntu snapshot: the environment is not yet fully bit-for-bit reproducible.
AprilTag, OpenCV, and Ceres are locked by source and hash.

## Diagnostic build

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

If the compiler does not support C++26, the fallback is explicit:

```sh
cmake --preset release -DCALIB_CXX_STANDARD=23
```

The C++23 fallback was built and tested with GCC 15.2 in `build/cxx23`, including
the camera–camera solver on October 17, 2026. This does not qualify an older compiler.
Using `-std=c++26` validates features used by this repository, not the compiler's
support for the entire future standard. Preset-build executables remain in their
build directory. A global `/usr/local` installation is unnecessary:
`scripts/install.sh` uses `~/.local` by default and accepts another explicit prefix.
Help and README distinguish delivered features from the complete design in `docs/`.

## Memory-error detection

```sh
cmake --preset sanitize
cmake --build --preset sanitize
ctest --preset sanitize
```

This preset also instruments the AprilTag source with AddressSanitizer and
UndefinedBehaviorSanitizer. CTest enables leak detection and stop-on-error; a UBSan
diagnostic cannot pass silently inside a successful CLI test. The initialization
adapter is instrumented, while local OpenCV and Ceres libraries remain uninstrumented
release builds; this is not a complete internal ASan/UBSan audit of those dependencies.
Results are documented in [docs/validation.md](docs/validation.md) and
[docs/initialization.md](docs/initialization.md).

Actually tested environment: Ubuntu 26.04.1 x86_64, GCC 15.2, CMake 4.2.3,
Ninja 1.13.2, CLI11 2.6.1, yaml-cpp 0.8.0, libpng 1.6.57, zlib 1.3.1,
nlohmann JSON 3.11.3, Python 3.14.4. Corresponding Debian/Ubuntu package versions
are recorded locally in `.deps/system-versions.txt`. `--sources-only` was also
replayed with verification of the cached archive.

## Rosbag extraction and validation (optional)

```sh
bash scripts/install_dependencies.sh --rosbag-tools
.deps/rosbag-venv/bin/python tests/rosbag_extraction.py
```

This mode does not use sudo. `scripts/setup_rosbag_tools.py` creates
`.deps/rosbag-venv` and uses a pip wheel whose URL/version/SHA-256 are locked in
`scripts/pip-bootstrap.json`, without depending on system pip/ensurepip. Libraries
and their hashes are locked in `requirements-rosbag.lock` and installed with
`--require-hashes --only-binary=:all:`. The current lock targets **CPython 3.14,
Linux x86_64**; the script rejects another environment instead of silently
installing different versions.

Optional-tool dependencies are rosbags 0.11.8, Pillow 12.3.0, NumPy 2.5.4, plus
transitive dependencies listed in the lock. No ROS environment is installed.
Converter tests are separate from CTest so building the C++ core does not make
these Python dependencies mandatory. Usage:
[docs/recorded-datasets.md](docs/recorded-datasets.md).

The RGGB8 `--bayer-conversion bilinear` extraction option and
`scripts/audit_extracted_dataset.py` use these same locked dependencies. Oxford
Spires requires no additional package or ROS installation.
