# cam_imu_calib

CLI-only tool under development for camera–camera and camera–IMU calibration,
including extrinsics, temporal synchronization, rolling shutter, and PDF reports.
Linux, existing recordings, and no ROS component in the core.

**Status: checks, detection, camera–camera calibration, and a first GS camera–IMU solver.**
Metric target geometry and 2D–3D correspondences are also exported. The
[optical models](docs/camera-models.md), pinhole-radtan and equidistant, provide
tested projections and Jacobians connected to camchain import. `calib_cameras`
jointly estimates intrinsics and extrinsics, then produces a camchain and PDF.
Its quality remains provisional: complete observability and uncertainty are not
yet computed. `calib_imu_camera` estimates GS camera extrinsics and offset, with
constant biases or piecewise-linear bias trajectories when IMU random walks are
provided, and fixed intrinsics. It exports local extrinsic/offset covariance after
marginalizing trajectory, biases, and gravity. Rolling shutter and caching remain
to be implemented.

## Installation

See [INSTALLATION.md](INSTALLATION.md) for dependencies, sudo password entry, and
versions. From the repository root:

```sh
bash scripts/install.sh
~/.local/bin/calib_cameras --version
~/.local/bin/calib_imu_camera --version
```

The script installs dependencies, builds release, runs the tests, then installs
both commands, the report renderer, and examples under `~/.local` by default.
Options include `--prefix /absolute/path`, `--jobs N`, `--skip-dependencies`, and
`--skip-tests`. It uses the explicit C++23 fallback for compatibility with common
Ubuntu compilers; `--cxx-standard 26` selects C++26 on a supporting toolchain.
AprilTag 3.4.5 is built from an official archive locked by commit and SHA-256.

For an uninstalled development build, CMake presets remain available:

```sh
bash scripts/install_dependencies.sh
cmake --preset release
cmake --build --preset release
ctest --preset release
```

## Native dataset

```text
session/
  cam0/1700000000000000000.png
  cam0/1700000000033333333.png
  cam1/1700000000000000000.png
  ...
  imu0/imu0.csv
```

The CSV contains `timestamp_ns,wx,wy,wz,ax,ay,az`: rad/s and m/s², with specific
force including gravity. Column order may vary if names remain exact and unique.
Timestamps are nonnegative int64 nanoseconds; CSV timestamps must be strictly
increasing. Images are sorted numerically. No camera CSV, manifest, or mandatory
`sensors.yaml` is needed.

Currently supported: PNG mono8/mono16/RGB8, including 3840×2160. For detection,
mono16 is converted with `floor(value / 256)`, without gamma or per-image
normalization; source PNGs remain unchanged. Palettes, alpha, and RGB16 are rejected.
A [separate ROS1 converter](docs/recorded-datasets.md) extracts bags into this
native format without a ROS dependency in the core. MCAP and direct capture remain
out of scope.

October Oxford Spires acquisitions were checked separately. At the user's request,
only `dataset/oxford-spires-2024-10-29/camera-imu` (cam0 + IMU) is retained locally.
Sensor timestamps are preserved; Bayer mosaics are explicitly converted to RGB8
PNG. The user confirmed the same target as DTVI 512/1024, shared in
[`examples/dtvi_aprilgrid.yaml`](examples/dtvi_aprilgrid.yaml). Detection, optical
initialization, excitation diagnostics, and a first GS camera–IMU calibration were
run. The estimate remains provisional without an independent validation acquisition;
its local covariance covers neither fixed intrinsics nor model errors. See the
[import protocol and limitations](docs/recorded-datasets.md#oxford-spires-2024-10-29).

## Available commands

Calibrate synchronized global-shutter cameras without supplied initial placement:

```sh
build/release/calib_cameras --dataset dataset/1024 \
  --target dataset/dtvi_aprilgrid.resolved.yaml --models pinhole-equi \
  --quad-decimate 4 --output dataset/calibration-1024
```

This command detects the target, initializes and refines intrinsics and relative
placement, checks reserved views, and exports `camchain.yaml`, `calibration.json`,
and `report.pdf`. It requires at least two cameras with exactly shared timestamps
and a connected shared-view graph. Use a new output directory. See the
[method and limitations](docs/camera-camera.md).

Check every image, timestamp, and decode:

```sh
build/release/calib_cameras --dataset ./session --check-dataset
```

Also check the IMU CSV, its statistics, and raw temporal overlap:

```sh
build/release/calib_imu_camera --dataset ./session --check-dataset
```

Detect the target and export observations:

```sh
build/release/calib_cameras --dataset ./session --target examples/aprilgrid.yaml --detect-only --output ./detections
```

Adapt the **family, border, physical dimensions, and IDs** to your target. The
example describes a native AprilTag 3 target. Delivered families are `tag36h11`
and `tagStandard41h12`, with their native border (`tagBorderBits: 1`). The
historical two-cell border is also supported for `tag36h11` with
`tagBorderBits: 2` and was tested on the 512/1024 recordings. Family and border
are never silently inferred from an incomplete YAML file.

Detect, then estimate initial intrinsics and target poses in one command:

```sh
build/release/calib_cameras --dataset dataset/1024 \
  --target dataset/dtvi_aprilgrid.resolved.yaml \
  --initialize-only --models pinhole-equi --quad-decimate 4 \
  --output dataset/initialization-1024
```

Use a new output directory. `pinhole-radtan` is also available. One model applies
to all cameras; multiple values must follow the exact selected-camera order. This
stage assumes global shutter and rejects `--cam` in `--initialize-only` mode so
neither supplied intrinsics nor an RS delay is silently ignored. Initial estimates
are not rig/IMU calibration. See the
[method, thresholds, and limitations](docs/initialization.md).

| Option | Current behavior |
| --- | --- |
| `--cameras cam0 cam1` | Explicit selection and order; otherwise numeric directory order |
| `--initialize-only` | Detection and initial intrinsic/target-pose estimation, without inter-sensor extrinsics |
| `--models pinhole-equi` | Required for calibration/initialization; `pinhole-radtan` also available |
| `--init-max-views N` | 10–200 initial views per camera and synchronized rig groups, default 60; one in five reserved for validation |
| `--cam camchain.yaml` | Checks intrinsics/resolutions; GS VI requires fixed intrinsics and zero `line_delay` |
| `--imu imu.yaml` | VI requires noise densities; positive random walks enable variable biases, otherwise biases are constant |
| `--vi-knot-interval S` | VI spline spacing, default 0.05 s; test sensitivity to a smaller spacing |
| `--vi-bias-knot-interval S` | Piecewise-linear bias spacing, default 0.1 s; used only with two positive random walks |
| `--vi-max-offset S` | Absolute camera–IMU offset bound, default 0.05 s |
| `--vi-pixel-sigma PX` | Visual-corner standard deviation for VI whitening, default 0.8 px |
| `--threads N` | Global image/initialization/Ceres budget; `0` = hardware concurrency, then reduced for the memory budget |
| `--memory-budget-mb N` | **Estimated** image-work budget in MiB, default 1024; not a strict RSS limit |
| `--tag-family NAME` | Geometry-free diagnostic alternative to `--target`, detection only |
| `--tag-border-bits N` | With `--tag-family` only; 1 native (default), 2 historical tag36h11 |
| `--quad-decimate N` | Detector decimation 1–4, default 1; corners are exported at original resolution |
| `--output DIR` | New directory, never overwritten; required for detection/calibration/initialization, optional for checks |
| `--help`, `--version` | Help and detector provenance |

`line_delay` is signed, in seconds per row, using
`t_imu = t_image + timeshift_cam_imu + y * line_delay`. The `lineDelay` alias is
accepted; conflicting values are rejected. Readout duration is
`(height - 1) * abs(line_delay)`. Inspection validates import without estimating
temporal parameters. The VI solver estimates offset with `line_delay = 0` and does
not import camchain extrinsics. Extended RS conventions and line-delay estimation
are not yet supported.

## Current outputs and diagnostics

- Summary JSON on stdout; progress and errors on stderr.
- `summary.json` when `--output` is supplied: rates, intervals, resolutions, IMU
  statistics, per-camera counts, parameters and detector version, duration, and
  process peak RSS.
- `detections.jsonl`: one record per image with integer timestamp, corners, IDs,
  Hamming distance, decision margin, and rejected tags with out-of-ID-range reason.
  A frame without tags remains present. Order is stable across worker counts.
- With `--target`, `target_geometry.json` describes metric 3D target points;
  `observations.jsonl` maps their IDs to observed pixels with structural rejections.
  See [conventions and limitations](docs/observations.md).
- `error.json` when an error occurs after directory creation. An `.incomplete`
  file is never promoted to a complete result after an error or handled interruption.
  This version has no automatic resume or cache.
- With `--initialize-only`, `initial-camchain.yaml` contains initial intrinsics and
  distortion coefficients; `initialization.json` contains `T_cam_target` poses,
  fitting and validation RMS, rejections, and limitations. No substitute rig/IMU
  transform, offset, or covariance is exported.
- Camera–camera calibration adds `camchain.yaml` with `T_cn_cnm1`,
  `calibration.json` with `T_cam_reference`, optical centers in meters, residuals,
  and validation views, then `report.pdf` with matrices and histograms. Identity
  of the first camera fixes the reference frame. Unknown covariance remains `null`;
  no offset or RS delay is estimated.

Checks decode every PNG and detect corrupt payloads, resolution changes, duplicates,
and non-finite/disordered CSV data. During detection, a complete absence of tags
for **one selected camera** is reported as insufficient data. Corner count and
AprilTag margin are not calibration scores. The solver adds provisional pose-
diversity, connectivity, and reprojection checks; they do not certify observability,
accuracy, or fitness for use.

Exit codes: `0` operation completed, `2` invalid input/arguments, `3` insufficient
data, `4` detector error, `5` I/O or runtime error, `8` unimplemented feature,
`130` handled interruption. Code 0 for checks means structural checks passed, not
that the data supports good calibration.

## Global-shutter camera–IMU calibration

```sh
build/release/calib_imu_camera --dataset ./session --cameras cam0 \
  --cam camchain.yaml --imu imu.yaml --target aprilgrid.yaml --output ./vi-result
```

This path detects the target, then jointly estimates a continuous trajectory,
`T_cam_imu`, `timeshift_cam_imu`, gravity, and IMU biases. Biases are constant when
random walks are absent or zero, otherwise piecewise linear. Camchain intrinsics
remain fixed; replace `--cam` with `--models pinhole-equi` to initialize them
automatically. `--imu` requires explicit noise densities. One camera is supported
and `line_delay` must be zero. Outputs include `camchain.yaml`, `calibration.json`,
and `report.pdf`, with marginalized local covariance and a numerical target-
observability diagnostic. Quality remains provisional.

The PDF has twelve pages following Kalibr's report structure: parameters, 3D IMU
trajectory with pose frames, position/orientation over time, IMU rate,
accelerometer/gyroscope measurements and predictions, inertial residuals, estimated
bias curves, signed x/y residuals for every evaluated corner over time, and a
time-colored x/y cloud. Pages are rendered directly by Python/Matplotlib with the
`Agg` backend; the CLI remains non-interactive and publishes the PDF only after a
complete valid render.

`calibration.json` also exposes an explicitly candidate HC0 sandwich estimate: the
100-trial campaign shows worse joint coverage, so it is not used by default.
An experimental temporal bootstrap can be requested with
`--vi-bootstrap-replicates` and `--vi-bootstrap-block-seconds`; it produces
`bootstrap.json` without replacing primary covariance. See the
[model, options, validation, and limitations](docs/camera-imu.md).

Two reproducible synthetic campaigns compare conditional covariance, its sandwich
candidate, and sensitivity to fixed optical parameters:

```sh
python3 scripts/qualify_vi_covariance.py --trials 100 \
  --output build/vi-covariance-monte-carlo.json
python3 scripts/qualify_vi_covariance.py --trials 30 \
  --bootstrap-replicates 32 --bootstrap-block-seconds 1 --timeout 120 \
  --output build/vi-bootstrap-pilot-1s.json
python3 scripts/benchmark_vi_intrinsics_sensitivity.py \
  --output build/vi-intrinsics-sensitivity.json
```

They use `build/release/vi_test_bridge`, do not replace an independent real
acquisition, and never interpret convergence as evidence of accuracy.

## Documents

- [Installation](INSTALLATION.md) and [dependency provenance](docs/dependencies.md)
- [Initial synthetic validation](docs/validation.md)
- [Real datasets, extraction, and regressions](docs/recorded-datasets.md)
- [Geometry and correspondences](docs/observations.md)
- [Intrinsic/pose initialization and real results](docs/initialization.md)
- [Camera–camera calibration, results, and limitations](docs/camera-camera.md)
- [GS camera–IMU calibration, model, and limitations](docs/camera-imu.md)
- [Tool and source survey](docs/research.md)
- [Architecture](docs/architecture.md) and [engine choice](docs/technical-choices.md)
- [Target CLI](docs/cli.md)
- [Dataset](docs/dataset.md) and [internal interfaces](docs/interfaces.md)
- [Rolling shutter and camera–IMU calibration](docs/rolling-shutter.md)
- [Roadmap](docs/roadmap.md) and [repository instructions](AGENTS.md)
