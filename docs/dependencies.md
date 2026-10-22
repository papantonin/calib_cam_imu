# Bundled dependency provenance

The current integration does not copy algorithms from Kalibr or Basalt.
The optical models are implemented locally; OpenCV initializes poses and Ceres
solves the joint camera–camera problem. Basalt remains a spline candidate.

AprilTag is built from the exact official source recorded in `dependencies.json`.
Its `LICENSE.md`, inspected in the archive, is BSD-2-Clause. Upstream notices
remain in the source tree extracted by CMake, including those covering the compiled
C files. The small-fixture generator calls the `apriltag_to_image` API.

System libraries are imported through their CMake targets. The following notices
were inspected in `/usr/share/doc/<package>/copyright` on the development machine:

| Component | Inspected package notice |
| --- | --- |
| CLI11 | BSD-3-Clause |
| yaml-cpp | X11 (package permission text) |
| nlohmann JSON | Expat, with notices for internal components |
| libpng | PNG Reference Library and component notices, as listed in the file |
| zlib | zlib and component notices, as listed in the file |

This development stage does not produce a standalone binary distribution. When
adding a distribution package, include the complete notices corresponding to the
versions and components actually distributed. This repository does not modify the
AprilTag source; apt package versions remain recorded locally.

## Optional ROS1 converter

Rosbags 0.11.8: Apache-2.0 license verified in the installed wheel metadata;
its AnyReader/rosbag1 API remains separate from the C++ core. Pillow 12.3.0 declares
MIT-CMU; complete notices for the wheels and transitive dependencies remain under
`.deps/rosbag-venv/lib/python3.14/site-packages/*dist-info/`.
Exact artifact versions and hashes are stored in `requirements-rosbag.lock`.
The pip bootstrap has its own lock in `scripts/pip-bootstrap.json`.

## Initialization and optimization

OpenCV 5.0.0: Apache-2.0 license inspected in `LICENSE`, retained in the locked
archive and local source tree. Ceres 2.2.0: BSD-3-Clause license inspected in
`LICENSE`, with third-party notices under `internal/ceres/miniglog` for the minimal
logging backend. These libraries remain locally built; their algorithms are not
copied into the project sources. Versions, commits, and SHA-256 hashes are recorded
in `dependencies.json`, and build options in the `setup_*` scripts.
Eigen comes from the `libeigen3-dev` package; the complete package notice must
accompany any future distribution. Poppler is invoked only as an independent PDF
consumer in tests and is not linked into the executables.
