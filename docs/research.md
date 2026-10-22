# Technical survey — October 15, 2026

Literature review based on the primary sources below. No comparative benchmark
has yet been run. Authors' claims are not independent validation on our acquisitions.

| Tool | Contribution to the project | Limitation or decision |
| --- | --- | --- |
| Basalt Headers | Camera models, splines, preintegration, calibration types | Mathematical foundation, not a complete calibration application |
| Basalt | Camera then IMU initialization/calibration example | Orchestration must be redesigned for a fully automatic CLI |
| Kalibr | Functional reference, YAML conventions, diagnostics | Its documented RS tool is monocular and does not simultaneously calibrate the IMU |
| Reprojection | Modern inputs, Aprilgrid3, automatic PDF | README says VI temporal and RS calibration are still under development |
| mrcal | Projection-uncertainty analysis | Methodological reference for going beyond RMS |
| iKalibr | Targetless multi-sensor spatiotemporal calibration | Alternative for later study; broader scope than our first version |
| OpenMVIS | Joint multi-camera/IMU/RS estimation, claimed Kalibr exports | Direct competitor to include in trials; maturity needs experimental verification |

## Basalt

The [requested repository](https://github.com/VladyslavUsenko/basalt-headers) is a
mirror whose page points to GitLab for contributions. Its
[README](https://github.com/VladyslavUsenko/basalt-headers/blob/master/README.md)
describes a header-only library and vcpkg integration with external Eigen, Sophus,
and cereal. Its declared license is BSD-3-Clause.

The [Se3Spline](https://github.com/VladyslavUsenko/basalt-headers/blob/master/include/basalt/spline/se3_spline.h)
uses separate SO(3) and R³ representations. The Ceres adapter must respect this
convention rather than assuming a coupled cumulative SE(3) spline. The
[Basalt calibration documentation](https://github.com/uw-visual-odometry/basalt/blob/master/doc/Calibration.md)
provides camera then camera–IMU examples on public datasets.

## Kalibr and alternatives

The [Kalibr README](https://github.com/ethz-asl/kalibr) describes its multi-camera
and visual-inertial capabilities. Its
[rolling-shutter page](https://github.com/ethz-asl/kalibr/wiki/Rolling-Shutter-Camera-calibration)
states the monocular-without-IMU restriction. Its
[YAML formats](https://github.com/ethz-asl/kalibr/wiki/yaml-formats) define transform
and temporal-shift signs that we must reproduce. The local official
`IccUtil.generateReport` generator and its `IccPlots` plots were also inspected
under their BSD license: 6.4×4.8-inch format, 3D trajectory with frames, IMU rate,
inertial predictions/errors/biases, and reprojections. Our renderer reimplements
this structure using data actually supplied by our solver; it copies no Kalibr
code and does not claim to reproduce Kalibr's time-varying bias spline.

[Reprojection](https://github.com/reprojection-calibration/reprojection) produces
a PDF and uses Aprilgrid3, distinguishing it from the historical AprilGrid. Its
code is declared AGPL-3.0: functional inspiration does not imply code reuse.

[mrcal projection uncertainty](https://mrcal.secretsauce.net/uncertainty.html) is
a useful reference for diagnostics outside actually observed points.
[iKalibr](https://github.com/Unsigned-Long/iKalibr) and its
[paper](https://arxiv.org/abs/2407.11420) offer a targetless direction.
[OpenMVIS](https://openmvis.com/) claims a GTSAM graph integrating RS readout,
extrinsics, and temporal parameters. Its published performance must be reproduced
before use as an acceptance reference.

For RS design, also see
[Continuous-Time Spatiotemporal Calibration of a Rolling Shutter Camera-IMU System](https://arxiv.org/abs/2108.07200)
and the [analysis of visual-inertial degeneracies](https://arxiv.org/abs/2201.09170).

## Library versions and choices

- **AprilTag 3**: the [latest exposed release](https://github.com/AprilRobotics/apriltag/releases/tag/v3.4.5)
  during the survey was `v3.4.5`. The [official project](https://github.com/AprilRobotics/apriltag)
  recommends `tagStandard41h12` for new uses. Compatibility with existing targets
  requires an explicitly configured family and geometry.
- **Ceres**: `2.2.0` is the latest stable tag shown on the
  [inspected tags page](https://github.com/ceres-solver/ceres-solver/tags).
  The [history](https://ceres-solver.readthedocs.io/latest/version_history.html)
  distinguishes development changes; “latest” documentation is not a release.
  Select a verified release or exact commit after testing, never a floating branch.
  See [sparse solvers and solve options](https://ceres-solver.readthedocs.io/latest/nnls_solving.html).
- **C++**: the [March 2026 report](https://herbsutter.com/2026/03/29/c26-is-done-trip-report-march-2026-iso-c-standards-meeting-london-croydon-uk/)
  announces completion of technical work on C++26. This does not guarantee complete
  support in every compiler and standard library. Target C++26 with verified
  features; document the C++23 fallback for the selected platform.
- **VI PDF**: [Matplotlib `PdfPages`](https://matplotlib.org/stable/api/backend_pdf_api.html#matplotlib.backends.backend_pdf.PdfPages)
  writes a multipage document directly. The non-interactive `Agg` backend is
  selected before `pyplot`. `matplotlib-cpp` was rejected after inspection: its
  [README](https://github.com/lava/matplotlib-cpp/blob/master/README.md) describes
  a wrapper around Python Matplotlib, so it removed no Python dependency and added
  interpreter linkage to the C++ binary.

The version numbers above are observations from sources on the survey date, not
yet project dependency locks. Basalt/Sophus/Eigen, Ceres, and compiler compatibility
still need to be built and tested together.

## Extraction of supplied recordings (October 16–17, 2026)

- [Rosbags AnyReader](https://ternaris.gitlab.io/rosbags/topics/highlevel.html):
  embedded definitions and serialization; a reader is not shared across workers.
- [Rosbags ROS1](https://ternaris.gitlab.io/rosbags/topics/rosbag1.html): Reader/Writer
  API, also used to create synthetic bags in tests.
- [PyPI metadata](https://pypi.org/project/rosbags/0.11.8/): version 0.11.8,
  Apache-2.0 license; integration confined to the optional converter.
- libpng API and AprilTag definition inspected in installed/locked sources for
  mono16 preservation and explicit adaptation of the historical tag36h11 border.

Actual protocol and conclusions: [recorded-datasets.md](recorded-datasets.md).

## Optical models (October 17, 2026)

- [OpenCV radial/tangential projection and distortion](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html):
  coefficient convention and `projectPoints` Jacobians.
- [OpenCV fisheye model](https://docs.opencv.org/4.13.0/db/d58/group__calib3d__fisheye.html):
  four-coefficient angular polynomial, skew fixed to zero in our model. Formulas
  are reimplemented without copying third-party code. System Python OpenCV 4.10.0
  serves only as an independent test oracle and is not linked to the C++ core.
  Conventions and limitations: [camera-models.md](camera-models.md).

## Intrinsic initialization (October 17, 2026)

- [Official OpenCV 5.0.0 release](https://github.com/opencv/opencv/releases/tag/5.0.0):
  stable release marked latest by the GitHub API, published June 6, 2026; resolved
  commit `40738fb16ceddb5fb3fea747585f7ce6abb0605b`. The 4.x branch also offers
  4.14.0; it is not confused with the new 5.x generation.
- [4→5 migration](https://github.com/opencv/opencv/wiki/OpenCV-4-to-5-migration):
  separate `calib` and `geometry` modules. APIs/flags and Apache-2.0 license were
  checked in the exact archive before integration. `calibrateCamera`,
  `fisheye::calibrate`, `solvePnP`, and point inversion are used behind the
  initialization adapter. No code was copied. Our optical models recompute
  residuals; Ceres now handles the joint rig problem and remains planned for VI/RS.

The C++ 5.0.0 backend and Python 4.10.0 oracle are distinct. Their coexistence does
not change dependencies of the optical models themselves. Experiments and limits:
[initialization.md](initialization.md).

## Camera–camera solver (October 17, 2026)

- [Official Ceres 2.2.0 tag](https://github.com/ceres-solver/ceres-solver/tree/2.2.0):
  latest stable tag verified in the tag list; GitHub's releases/latest API returns
  no release. Resolved commit `85331393dc0dff09f6fb9903ab0c4bfa3e134b01`.
  APIs, CMake options, and BSD-3-Clause license were inspected in the exact archive;
  master documentation may describe different dependencies, notably Abseil.
- [Ceres solving](https://ceres-solver.readthedocs.io/latest/nnls_solving.html):
  Huber, Schur grouping, and constant blocks. Current use: autodiff,
  `SPARSE_SCHUR`, `EIGEN_SPARSE`, fixed reference camera, shared GS timestamps.
- The camera–camera report retains the small in-house PDF 1.4 text/vector export,
  validated with Poppler. The richer VI report is rendered directly by Python/
  Matplotlib and `PdfPages`, without a C++ wrapper or Cairo/Pango dependency.

Evidence and limitations: [camera-camera.md](camera-camera.md). Basalt is not
required for the discrete GS solver; its value for VI/RS splines remains to evaluate.

## GS camera–IMU solver

- [Sommer et al., CVPR 2020](https://arxiv.org/abs/1911.08860): cumulative SO(3)
  spline and temporal-derivative recurrences. In-house implementation without
  copied third-party code; verified against analytical trajectories and finite
  differences in `tests/vi_residual.cpp`.
- [Ceres modeling](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html):
  dynamic autodiff and quaternion/sphere manifolds. The locked 2.2.0 API was
  checked in local headers before use; no dependency update.

## Oxford Spires import (October 17–18, 2026)

- [2024-10-29 delivery](https://huggingface.co/datasets/ori-drs/oxford_spires_dataset/tree/main/calibration/calibration-sequences-2024-10-29):
  archive already downloaded by the user; images are not redistributed. Types,
  formats, counts, and timestamps are checked directly in local bags, not inferred
  only from filenames.
- [Calibration documentation](https://huggingface.co/datasets/ori-drs/oxford_spires_dataset/blob/main/calibration/README.md)
  and [target description](https://huggingface.co/datasets/ori-drs/oxford_spires_dataset/blob/main/calibration/calibration_target.txt):
  separate camera–LiDAR and camera–IMU acquisitions. These descriptions are not
  sufficient to silently assign an AprilTag family or border to each bag.
- In-house bilinear Bayer RGGB8 conversion, with no copied third-party code: locked
  NumPy, preservation of all measured sites, independent neighbor/edge checks, and
  no radiometric correction. A manufacturer-pipeline preprocessed RGB extraction
  would be an alternative, but is unavailable in the supplied raw messages. The
  audit retains the Bayer payload fingerprint.
