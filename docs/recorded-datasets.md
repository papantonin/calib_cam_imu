# Recorded datasets: extraction and regression

Status as of October 17, 2026. The `dataset-calib-imu1_512_16.bag` and
`dataset-calib-imu1_1024_16.bag` files from `~/Downloads` were extracted into
`dataset/512` and `dataset/1024`. They are neither moved nor modified. The entire
`dataset/` directory is excluded from Git, including images, the supplied YAML,
observations, and local reports. Ordinary tests neither download a dataset nor
send one to an external service.

## Ready-to-use inputs

```text
dataset/
  dtvi_aprilgrid.yaml             # original supplied by the user, unchanged
  dtvi_aprilgrid.resolved.yaml    # same dimensions, explicit family/border
  512/
    cam0/<timestamp_ns>.png
    cam1/<timestamp_ns>.png
    imu0/imu0.csv
    provenance.json
    frames.jsonl
  1024/
    cam0/<timestamp_ns>.png
    cam1/<timestamp_ns>.png
    imu0/imu0.csv
    provenance.json
    frames.jsonl
  validation/
```

Each dataset contains **1,038 images per camera** and **10,345 IMU measurements**.
The images are lossless mono16 PNG files, respectively 512 × 512 and 1024 × 1024.
Camera duration is 51.851620 s at a mean rate of 19.9994 Hz; mean IMU rate is
199.367 Hz. Camera intervals range from 49.899 to 50.104 ms and IMU intervals
from 4.997 to 5.035 ms.

The IMU CSV and camera timestamps are identical across resolutions. The first
inspected 512 image exactly matches the 2 × 2 pixel average of the corresponding
first 1024 image; this pixel relationship was not checked for every pair.
**These datasets are not two independent captures** for evaluating calibration
generalization.

The IMU starts 11.263 ms after the first image pair. The check validates raw
overlap, not the presence of IMU measurements around every corner: the future
solver must choose its time window without silent extrapolation. No offset, gain,
axis rotation, bias, or gravity removal is applied. SI values from the ROS message
are preserved; their physical calibration remains to be checked during estimation.
The `/vrpn_client/raw_transform` stream is listed as ignored and is not presented
as extrinsic ground truth.

## Installation and extraction reproduction

```sh
bash scripts/install_dependencies.sh --rosbag-tools
.deps/rosbag-venv/bin/python scripts/extract_rosbag.py \
  --bag ~/Downloads/dataset-calib-imu1_512_16.bag --output dataset/512
.deps/rosbag-venv/bin/python scripts/extract_rosbag.py \
  --bag ~/Downloads/dataset-calib-imu1_1024_16.bag --output dataset/1024
```

These directories already exist on the workstation: the commands refuse to
overwrite them. Choose a new name to repeat the extraction. `--inspect` prints
topics without extracting. Automatic discovery is limited to `/camN/image_raw`
names and a single IMU; otherwise use
`--camera cam0=/topic --camera cam1=/other_topic --imu-topic /imu`.

The Python converter is separate from the C++ reader. It uses the documented
[Rosbags AnyReader](https://ternaris.gitlab.io/rosbags/topics/highlevel.html) APIs
to read embedded definitions without a ROS installation. The lock and local
environment are detailed in [INSTALLATION.md](../INSTALLATION.md). CLI support is
deliberately limited to ROS1 bag v2 and mono8/mono16/rgb8/bgr8 images.
`bayer_rggb8` is also accepted with an explicit `--bayer-conversion bilinear`.
Other encodings and compressed images are not converted implicitly.

Conversion contract:

- Frame names are built from `header.stamp.sec * 10^9 + nanosec` as integers.
  Reception timestamps are never silently substituted. The explicit
  `--timestamp-source bag` option preserves recording times with a warning and
  retains the original headers; it does not establish synchronization.
- No corrective sorting, duplicate removal, or overwriting: non-increasing sensor
  timestamps, changing dimensions, and non-finite IMU measurements cause failure.
- `step`, padding bytes, and mono16 endianness are respected. PNG is re-encoded
  without changing values; BGR8 is explicitly reordered to RGB8.
- One bag reader is used, with PNG written by workers and at most two pending
  tasks per worker. `--workers` controls this budget (default 4).
- Every written PNG is immediately decoded again and compared with the ROS pixels.
  `frames.jsonl` retains source/reception timestamps, encoding, dimensions, and
  SHA-256 hashes of canonical pixels and PNG. `provenance.json` retains the bag
  SHA-256, tool versions, topic mapping, and counters.
- The final directory is published only after success; errors retain an
  `.incomplete` directory and `error.json`. There is no automatic resume mechanism.

These audit metadata are optional for `calib_*`: the native format still requires
no manifest.

## Oxford Spires 2024-03-19 — waiting for valid timestamps

The local `~/Downloads/calibration-sequences-2024-03-19` directory contains 166
ROS1 bags. A complete inventory and reading of every camera message gives:

| Topic | Image messages | Resolution / ROS encoding | Zero timestamp headers |
| --- | ---: | --- | ---: |
| `/alphasense_driver_ros/cam0` | 700 | 1440 × 1080 / bgr8 | 700 |
| `/alphasense_driver_ros/cam1` | 599 | 1440 × 1080 / bgr8 | 599 |
| `/alphasense_driver_ros/cam2` | 400 | 1440 × 1080 / bgr8 | 400 |

The only other topic is `/hesai/pandar` (1,660 point clouds). **No IMU message**
is present. Inspected images show a checkerboard, consistent with the
[camera–LiDAR target description](https://huggingface.co/datasets/ori-drs/oxford_spires_dataset/blob/main/calibration/calibration_target.txt).
This is not the AprilGrid target supported by the currently implemented path.
The [calibration documentation](https://huggingface.co/datasets/ori-drs/oxford_spires_dataset/blob/main/calibration/README.md)
also distinguishes the camera–LiDAR and camera–IMU captures.

User decision: **wait for data with valid sensor timestamps**, without replacing
zero headers with recording timestamps. The attempted export based on bag times
was interrupted and its temporary PNG files removed; no native Oxford dataset was
published and no fabricated IMU CSV was created. Source bags are unchanged. The
inventory and waiting status are retained in
`dataset/validation/oxford-spires-2024-03-19/` (excluded from Git).

The converter now accepts multiple paths or a directory through `--bag`. It orders
bags by recording interval, rejects overlaps, and opens only one data bag at a
time to bound chunk memory. `--without-imu` explicitly permits a camera-only
export; an IMU remains required by default. The native reader and its dependencies
are unchanged. Provenance format v2 lists sources and their SHA-256 hashes; each
frame retains its source bag, header timestamp, and recording timestamp.

Validation: seven synthetic converter tests pass, including unordered multiple
sources, overlap, missing IMU, zero headers, and explicit recording-time selection.
Both 512/1024 bags were re-extracted into temporary directories: for each, all
2,076 PNG files and the IMU CSV are byte-identical to the references. Pixels and
all reference IMU measurements were also audited against their bags. Local result:
`dataset/validation/spires-converter-regression.json`.

Resumption criterion: obtain data with valid sensor timestamps and an IMU stream
for VI calibration. Checkerboard support is a separate extension and is not
implemented by this import.

## Oxford Spires 2024-10-29

The [October delivery](https://huggingface.co/datasets/ori-drs/oxford_spires_dataset/tree/main/calibration/calibration-sequences-2024-10-29)
contains four distinct captures locally. The camera and camera–IMU captures are
extracted separately, using **header timestamps**:

```text
dataset/oxford-spires-2024-10-29/
  cameras/
    cam0/<timestamp_ns>.png
    cam1/<timestamp_ns>.png
    cam2/<timestamp_ns>.png
    provenance.json
    frames.jsonl
  camera-imu/
    cam0/<timestamp_ns>.png
    imu0/imu0.csv
    provenance.json
    frames.jsonl
```

The `cameras_calibration_2024-10-29-15-27-05.bag` bag contains 3,230 / 3,222 /
3,173 images for cam0/cam1/cam2 and no IMU. The
`cameras_imu_calibration_2024-10-29-15-37-18.bag` bag contains 1,882 cam0 images
and 37,613 IMU measurements. Topics are `/alphasense_driver_ros/camN` and
`/alphasense_driver_ros/imu`. The camera–LiDAR and intrinsic-IMU bags (10,822,031
measurements) remain in Downloads; their data are not mixed with another capture
to fabricate a synchronized dataset.

The 1440 × 1080 images use Bayer RGGB8, unlike the mono16 PNG files in the
512/1024 datasets. Explicit conversion produces RGB8 PNG: bilinear interpolation
of missing channels, normalized averaging of available neighbors at borders, and
round-to-nearest integer (half up). Every measured sample is preserved in its
original channel. No white balance, gamma, rectification, exposure correction, or
resolution change is applied. The PNG is lossless relative to the **interpolated
RGB**, not a raw copy of the mosaic. The source bag remains intact; Bayer-payload
and PNG hashes and the method are recorded in metadata. Conversion uses the
already pinned NumPy dependency and adds no new dependency. It does not replace
radiometric calibration.

Reproduction (new output directories are mandatory):

```sh
.deps/rosbag-venv/bin/python scripts/extract_rosbag.py \
  --bag ~/Downloads/calibration-sequences-2024-10-29/cameras_imu_calibration_2024-10-29-15-37-18.bag \
  --camera cam0=/alphasense_driver_ros/cam0 --imu-topic /alphasense_driver_ros/imu \
  --bayer-conversion bilinear --output dataset/oxford-spires-2024-10-29/camera-imu
.deps/rosbag-venv/bin/python scripts/extract_rosbag.py \
  --bag ~/Downloads/calibration-sequences-2024-10-29/cameras_calibration_2024-10-29-15-27-05.bag \
  --camera cam0=/alphasense_driver_ros/cam0 --camera cam1=/alphasense_driver_ros/cam1 \
  --camera cam2=/alphasense_driver_ros/cam2 --without-imu \
  --bayer-conversion bilinear --output dataset/oxford-spires-2024-10-29/cameras
```

Native checks:

```sh
build/release/calib_imu_camera --dataset dataset/oxford-spires-2024-10-29/camera-imu --check-dataset
build/release/calib_cameras --dataset dataset/oxford-spires-2024-10-29/cameras --check-dataset
```

Reproducible independent audit for each subset:

```sh
.deps/rosbag-venv/bin/python scripts/audit_extracted_dataset.py \
  dataset/oxford-spires-2024-10-29/camera-imu \
  --output dataset/validation/oxford-spires-2024-10-29/camera-imu-audit.json
```

It rereads the bags and verifies their SHA-256 hashes, every timestamp, all IMU
values, all measured Bayer pixels, interpolated channels on a pixel sample that
includes every image border, PNG hashes, and counts. Conversion and auditing are
also covered by the converter's nine synthetic tests, including known RGB, random
data, row padding, borders, and PNG corruption. Re-extracting 512/1024 preserves
all 2,076 PNG files from each dataset and their IMU CSV files identically to the
references (`bayer-converter-regression.json`).

Native checks completed successfully: 9,625 PNG files decoded for `cameras` and
1,882 PNG files plus 37,613 measurements for `camera-imu`. Every exported header
is nonzero and strictly increasing within its stream. The camera dataset has
3,159 timestamps shared by all three sensors. Different counts are not compensated
by duplicating frames; intervals reach 0.3 s on cam1/cam2. The VI dataset spans
approximately 94.1 s at 20 Hz camera / 399.4 Hz IMU. The IMU starts 35.4219 ms
after the first image and ends 103.6659 ms after the last one: the future solver
must choose a covered window without silent extrapolation. These boundary gaps
are not desynchronization estimates.

Extraction took 97.47 s (VI) and 397.56 s (three cameras), using four workers;
these are indicative times under workstation load, not an isolated benchmark.
The C++ check took 19.24 s / 63.98 s respectively, with observed peak RSS of
14,032 / 17,376 KiB. Check reports are under the `camera-imu-check` and
`cameras-check` subdirectories of
`dataset/validation/oxford-spires-2024-10-29/`. Counts and temporal intersections
are recorded in `timestamps.json`. Both independent audits passed:
`camera-imu-audit.json` (93.97 s, sequential PNG reading) and `cameras-audit.json`
(141.26 s, four workers with at most eight pending checks). These different
datasets do not provide a comparative speedup measurement. Every selected file
and measurement was checked; no reception timestamp replaced a header.

These checks validate the import, not a calibration. At the time of this import,
the camera–IMU solver remained to be developed. On October 18, 2026, the user
confirmed the same AprilGrid as DTVI 512/1024: 6 × 6 tags of 0.088 m, relative
spacing 0.3, tag36h11, historical two-cell border, IDs 0–35. The shareable
configuration is [`examples/dtvi_aprilgrid.yaml`](../examples/dtvi_aprilgrid.yaml),
equivalent to the locally resolved YAML; original user YAML files remain unchanged.
Use `--target examples/dtvi_aprilgrid.yaml` for future Oxford detection and
calibration trials, without inferring success from target confirmation alone.
The effect of demosaicing on corner accuracy remains to be evaluated during this
calibration validation; conversion tests do not measure that bias.

### Optical validation and excitation of the camera–IMU dataset — October 18, 2026

At the user's request, only the `camera-imu` subset is retained locally; the bag
and directories associated with the other cameras were deleted after their audit.
Optical initialization was run with:

```sh
build/release/calib_imu_camera \
  --dataset dataset/oxford-spires-2024-10-29/camera-imu \
  --target examples/dtvi_aprilgrid.yaml \
  --initialize-only --models pinhole-equi --quad-decimate 4 \
  --init-max-views 60 --output <new-directory>
```

Runs with one and four workers produce byte-identical `detections.jsonl`,
`observations.jsonl`, `target_geometry.json`, `initialization.json`, and
`initial-camchain.yaml` files. With four workers, 1,882 images were processed in
40.29 s with 32,400 KiB peak RSS; with one worker, 94.40 s and 17,368 KiB. These
single timings under load are not a benchmark.

The resulting equidistant estimate is `[fx, fy, cx, cy] = [697.365, 696.206,
714.510, 558.025]` px. The 38 fitting views give 0.492 px RMS and the nine held-out
views 2.475 px RMS. Sampled distortion remains locally invertible over the observed
field; 170 fitting corners were rejected. One held-out view nevertheless reaches
7.743 px: the result remains a `not_certified` initialization, not a final
calibration.

The 26,316 accepted corners cover 106 cells of a 12 × 9 image grid, and their
convex hull covers 93.6% of the image. Actually selected views cover 99 cells and
84.2%. Laplacian variance was measured as a relative sharpness indicator, without
an MTF threshold or exposure metadata. The view with the largest residual lies
at the 36.9th percentile of this metric; its error is therefore not explained by
exceptional blur according to this indicator alone.

Two additional initializations on the temporal halves give a second-minus-first
difference of `[-1.874, -1.600, +0.873, +2.599]` px on `[fx, fy, cx, cy]`, less
than 0.47% per parameter. The second half has fewer usable poses and a higher
held-out RMS (3.079 px versus 1.844 px); distortion coefficients vary more. This
consistency is useful but does not replace covariance or ground truth. Against
the `cam0.yaml` published by Oxford Spires, intrinsic differences remain between
-2.55 and +1.75 px, at most 0.455%. The external reference may come from different
sequences and a different demosaicing pipeline; it is not treated as ground truth.

Over the strictly shared interval, 37,571 IMU measurements cover 94.063 s. No
interval exceeds 1.5 times the 2.5036 ms median. Angular standard deviations are
`[1.946, 2.047, 1.109]` rad/s and the three covariance eigenvalues are
`[1.213, 3.789, 4.207]` (rad/s)²: rotation is not confined to one axis. The share
of measurements above 1 rad/s is 54.0%. Accelerometer standard deviations are
`[3.063, 2.919, 3.016]` m/s². With the explicit heuristic
`||omega|| < 0.05 rad/s` and `abs(||a|| - 9.80665) < 0.15 m/s²`, the longest
contiguous stationary period lasts only 47.6 ms; this dataset therefore does not
replace a static bias or Allan-deviation capture.

Finally, over 46 intervals between selected visual poses, the net rotation angle
from the poses and the angle obtained through trapezoidal gyro integration have a
Pearson correlation of 0.99987. Median absolute difference is 0.00385 rad and P99
is 0.01498 rad. This angle comparison is invariant to a fixed extrinsic rotation,
but remains exploratory: spaced poses, biases, offset, rolling shutter, and vector
directions are not estimated. It establishes motion consistency and enough
excitation to develop the solver, not observability or final suitability for VI
calibration.

Unversioned private artifacts are `vi-gs-final/initialization.json`,
`camera-optical-diagnostics.json`, and `camera-imu-excitation.json`. The published
`cam0.yaml` and `imu.yaml` files are retained in `published-reference/` with their
SHA-256 hashes. The next validation must begin with synthetic recovery of the
future camera–IMU solver before comparing its extrinsics with published values.
That step has now been run below.

### First GS camera–IMU solver — October 19, 2026

The complete CLI path passed on 1,882 cam0 images and 37,613 IMU measurements,
with the explicitly confirmed target, `--models pinhole-equi`, decimation 4,
two threads, and `--vi-knot-interval 0.025`. Private outputs are under
`dataset/oxford-spires-2024-10-29/vi-gs-final/`: completed `calibration.json`,
`camchain.yaml`, `report.pdf`, and `summary.json`. The PDF was checked with
`pdfinfo` and `pdftotext`; the camchain exports IMU to camera and a signed offset
according to `t_imu = t_cam + timeshift_cam_imu`.

The longest retained continuous visual window spans 38.599 s, from 7.200 to
45.799 s relative to the first image; exclusions and their reasons are retained.
The fit uses 16,276 corners and 15,338 IMU measurements; 4,348 corners from
held-out frames evaluate the trajectory without re-estimating their poses.

| Metric | Result |
| --- | ---: |
| Camera–IMU offset | +2.07525 ms |
| Held-out reprojection median / RMS / P95 | 0.380 / 2.791 / 4.925 px |
| Fitting reprojection RMS | 2.558 px |
| Vector gyro residual RMS | 0.01984 rad/s |
| Vector accelerometer residual RMS | 0.13339 m/s² |
| Translation difference from the published October reference | 2.464 mm |
| Rotation difference from that reference | 0.2493° |
| Total duration / peak RSS, single measurement | 145.0 s / 161,992 KiB |

The IMU profile used for this result comes from
`published-reference/october/imu.yaml`, not the previously downloaded generic
file (June profile). The comparison uses `cam0.C_q_CI` (xyzw order) and
`cam0.C_r_CI` from the October `cam-lidar-imu.yaml`; these extrinsics are not used
for initialization. Sources are the files under
[calibration-sequences-2024-10-29](https://huggingface.co/datasets/ori-drs/oxford_spires_dataset/tree/03f4382308333aa70c3253f12acd3fbf0c7c4a15/calibration/calibration-sequences-2024-10-29),
revision `03f4382308333aa70c3253f12acd3fbf0c7c4a15`. Hashes and local measurements
are in `vi-gs-final/validation.json`. The external reference is not metrological
ground truth; no reference offset is claimed.

A trial with a 0.1 s interval exposed an overly coarse model. Its intermediate
artifacts were removed during dataset cleanup. Temporary visual-initialization
anchors and final inertial-residual checks now prevent publishing this case based
on convergence alone. The 0.05 and 0.025 s trials, both using the same old June
noise profile, differ by 1.434 mm, 0.0737°, and 0.423 ms; gyro RMS falls from
0.08745 to 0.01979 rad/s. These trials motivate the explicit chosen interval,
without silently changing the 0.05 s default or establishing universal accuracy.

The result remains **provisional**: fixed intrinsics, constant biases, visible
outliers in RMS and P95, checking on the same capture, and unavailable global
observability and ground truth. Because the private 512/1024 datasets were not
locally present, no new VI validation is claimed on them. The model, tests, and
reproducible command are in [camera-imu.md](camera-imu.md).
The second `--cam` path recovers the same offset and extrinsics within 1.39e-17
after harmonizing the Eigen allocator. Detections and observations remain
identical; duration is 126.4 s and RSS 160,780 KiB, under different concurrent
load. Its strictly redundant directory was then removed; useful metrics and hashes
are consolidated in `vi-gs-final/validation.json`.

### Local VI covariance — October 20, 2026

The `--cam` path was replayed with targeted covariance under
`dataset/oxford-spires-2024-10-29/vi-gs-covariance-validation/`. The central
estimate is unchanged: maximum `T_cam_imu` difference of 2.78e-17 and strictly
identical offset to the previous run. The final replay takes 73.64 s and 298,156
KiB maximum RSS; covariance alone takes 1.19 s for 124,580 residuals, 9,278
nuisance dimensions, and 6,579,012 nonzero coefficients.

Local standard deviations are `[0.108, 0.125, 0.066]` mrad for left rotation
perturbation, `[0.124, 0.124, 0.077]` mm for translation, and 8.29 µs for offset.
The nuisance block has twelve small pivots associated with boundary trajectory
controls. Schur complements regularized at `1e-8` and `1e-10` differ relatively
by 5.47e-5, below the versioned `1e-3` threshold; the 7×7 target information
retains full numerical rank, with a normalized spectral ratio of 0.591. An
exploratory 0.05 s-interval run retains this diagnostic but moves the offset to
1.687 ms and its local standard deviation to 12.9 µs. This model sensitivity is
greater than the conditional covariance conveys: these figures are neither
ground truth nor guaranteed precision.

After adding the experimental HC0 sandwich estimator, a `--cam` replay on the
same capture, stored outside the dataset in
`build/oxford-sandwich-validation-20261020/`, preserves `T_cam_imu` within
`1.39e-17` and an identical offset. The sandwich field is deliberately
`unavailable`: the actual nuisance block is `rank_deficient_target_stable` and
its nominal elimination uses the regularized limit, which this experimental
estimator excludes. The path takes 76.80 s, 298,700 KiB maximum RSS, and 1.34 s
for uncertainty. This unavailability is a scientific safeguard, not a failure of
the nominal covariance.

### Four-thread / automatic-selection reproducibility — October 21, 2026

Two complete `--models pinhole-equi` paths, from AprilGrid detection to extrinsic
covariance, were run sequentially with the same Oxford options
(`--quad-decimate 4`, 0.025 s spline). The machine has two physical cores and four
logical processors. Both `--threads 4` and `--threads 0` therefore used four
workers.

| Setting | Wall time | User CPU time | Peak RSS | Result |
| --- | ---: | ---: | ---: | --- |
| `--threads 4` | 53.39 s | 150.77 s | 310,996 KiB | converged |
| `--threads 0` | 56.09 s | 151.85 s | 311,288 KiB | converged |

Both runs find 6,594 tags in 1,074 images, retain 26,316 corners, and produce
byte-identical detection, observation, and initialization files. Extrinsic
differences are `1.16e-17` m in translation and `4.79e-18` rad in rotation; the
offset is strictly identical at `+2.075249786 ms`. Held-out reprojection remains
0.380 px median, 2.791 px RMS, and 4.925 px P95. Timings come from one measurement
per setting: the 2.70 s difference does not establish a systematic performance
difference. Private artifacts are under `vi-gs-threads4-20261021/` and
`vi-gs-threads-all-20261021/`, excluded from Git with the rest of `dataset/`.

### EuRoC dataset `dataset-calib-imu1_1024_16` — October 21, 2026

The dataset supplied under `Downloads` is already extracted: 1,038 mono16
1024×1024 PNG files for each of two cameras and one IMU CSV. Because the VI solver
is monocular, this test uses cam0 and the 10,345 IMU measurements. Adaptation to
the native format preserves PNG files through hard links and rewrites only the
CSV header; it takes 0.03 s and therefore does not measure copying or decoding
the 1.7 GB of data.

The fresh `--models pinhole-equi` path, without cache or initial extrinsics, uses
`examples/dtvi_aprilgrid.yaml`, `--quad-decimate 4`, a 0.025 s spline, and
`--threads 0`. All four logical processors are selected. The complete command—
decoding, AprilGrid detection, correspondences, optical initialization, VI
optimization, diagnostics, and exports—takes 52.86 s, 98.94 s of user CPU, and
327,336 KiB maximum RSS. The separate preliminary structural check takes 5.95 s.

Detection finds 14,939 tags in 1,022 images and retains 59,756 corners. Held-out
reprojection is 0.262 px median, 1.389 px RMS, and 2.324 px P95. Estimated offset
is `+46.355 µs`. Without using the supplied camchain as a seed, the extrinsic
differs from this reference by 1.565 mm and 0.02256°. This reference is not
independent ground truth.

The solver converges and exports its extrinsic, but correctly refuses local
covariance with status `nuisance_nullspace_couples_target`: the two nuisance-block
regularizations move the Schur complement by 0.0328, beyond the 0.001 limit.
Uncertainty therefore remains unknown instead of being replaced by zero. This
case provides the next regression for diagnosing gauges, boundary controls, and
spline-interval sensitivity.

After migrating the report to Python/Matplotlib, an initial replay with automatic
selection of the four logical processors took 36.38 s, 90.71 s of user CPU, and
327,256 KiB maximum RSS for the provisional five-page PDF. A fresh replay of the
Kalibr-aligned report takes 40.03 s, 102.09 s CPU, and 325,392 KiB RSS; it generates
twelve hybrid pages in the reference's 6.4×4.8-inch format. It plots 977
`T_world_imu` poses, 10,298 inertial samples, and 59,376 signed x/y residual pairs:
all corners from frames actually evaluated in the continuous window. The 3D
trajectory and its frames, six motion components, rate, inertial
measurements/predictions/residuals, constant biases, and two reprojection pages
were inspected after rasterization. `pdfinfo` reports twelve pages and `pdftotext`
reads them without diagnostics. Residuals reach 15.32 px in x and 16.06 px in y;
the report therefore preserves the full scale without hiding these outliers. The
61 unplotted frames are rejected before fitting or lie outside the selected window
and have no estimated pose. The historical C++ artifact is
`vi-report-v3-20261021/report.pdf`. The current artifact is
`dataset-calib-imu1-1024-16-cam0-vi-kalibr-report-20261021/report.pdf`;
`pdfinfo` identifies Matplotlib 3.6.3 as creator and no `.incomplete` file remains
after publication.

## Development driven by the data

Mono16 was previously rejected. It is now accepted; the detector receives the
high byte of each pixel (`floor(raw / 256)`), with no gamma or per-frame
normalization. Native data remain 16-bit. Independent tests cover limit values,
byte transitions, gamma chunks, and corrupted PNG files.

The user YAML specifies 6 × 6 tags, `tagSize: 0.088` m, and `tagSpacing: 0.3`,
but omits family and border. A native `tag36h11` search on both sequences found
no tag. Explicitly extending the black border to two cells detects IDs 0 through
35 on real images. The derived file therefore adds:

```yaml
tagFamily: tag36h11
tagBorderBits: 2
tagStartId: 0
```

The adapter preserves the official AprilTag codebook and shifts bit coordinates
by one cell, with a border width increased by two cells. This behavior is distinct
from the native family and is tested on independently built wide-border bitmaps,
four rotations, decimations 1/2/4, and geometrically expected corners. It does not
change the supplied physical size. The
[official tag36h11 definition](https://github.com/AprilRobotics/apriltag/blob/94be783968e5091bcc9972c72c84fd63efce2935/tag36h11.c)
remains unchanged in the archive; adaptation is local to the detector.

To explore a family without inventing geometry:

```sh
build/release/calib_cameras --dataset dataset/512 --detect-only \
  --tag-family tag36h11 --tag-border-bits 2 --output dataset/exploration
```

This mode reports unknown geometry and cannot be used for metric calibration.

## Regression to rerun after future changes

```sh
.deps/rosbag-venv/bin/python tests/rosbag_extraction.py
.deps/rosbag-venv/bin/python scripts/validate_recorded_datasets.py \
  --target dataset/dtvi_aprilgrid.resolved.yaml \
  --output dataset/validation/new-iteration
```

The result directory must be new. The script checks each bag's SHA-256, rereads
**all** selected messages, independently compares every PNG pixel, timestamp, and
IMU value, then runs checking and detection on both resolutions. JSONL files are
compared byte-for-byte between one and four workers. An assertion or nonzero exit
code fails; the result is written to `regression.json`, including on failure. Run
without `python -O` (assertions are part of the test protocol). Future features
must add their criteria and must not interpret these checks as solver validation.

To also check that raw detections remain unchanged, add
`--baseline dataset/validation/decimated-mono16-legacy` to the regression profile
with decimations 2/4. The script now also checks `target_geometry.json` and
`observations.jsonl`, their units, associations, and parallel stability. Results
for this increment are in [observations.md](observations.md).

Checking and detection can be run separately:

```sh
build/release/calib_imu_camera --dataset dataset/512 --check-dataset
build/release/calib_imu_camera --dataset dataset/1024 --detect-only \
  --target dataset/dtvi_aprilgrid.resolved.yaml --output dataset/detection-1024
```

## Results and limitations

The complete initial result is in
`dataset/validation/initial-mono16-legacy/regression.json`: **audit passed**, with
identical observations for one and four workers on each sequence. Release and
ASan/UBSan tests pass (4 CTest suites), as do the converter's 5 tests
(endianness/padding, BGR, invalid format, ROS1 round trip, and
duplicates/incompleteness).

At decimation 1, detection is highly sensitive to resolution:

| Dataset | Images with tags cam0 / cam1 (out of 1,038 each) | Total tags |
| --- | ---: | ---: |
| 512 | 800 / 810 | 3,894 |
| 1024 | 67 / 52 | 123 |

This result exposes a detector-configuration limitation, not proof that the 1024
images are unsuitable for calibration. Preliminary checks do not confuse
“reading and detection ran” with “high-quality calibration.” Search-scale trials
are retained under `dataset/probes/`; they motivate a second regression with
explicit decimation, without changing the global default based on this capture alone.

Performance profiles record time and `getrusage(RUSAGE_SELF).ru_maxrss`. Under
the Python audit launcher, this peak may include memory inherited before exec;
do not use these values as an isolated detector-memory measurement. Timings are
single measurements, not throughput guarantees.

No extrinsic is estimated, and no calibration RMS/covariance or PDF is produced
yet. Observability, useful geometric coverage, the optical model, and rolling-
shutter parameters require the future solver stages.

### Effect of decimation on the full sequence

Second regression: `dataset/validation/decimated-mono16-legacy/regression.json`.
The full audit passes again, with identical observations between one and four
workers. Parameters were explicitly selected after search-scale trials; the
default remains 1, with no hidden resolution adaptation.

| Dataset / decimation | Images with tags cam0 / cam1 | Total tags | Time, 4 workers |
| --- | ---: | ---: | ---: |
| 512 / 2 | 1,006 / 1,007 | 21,444 | 3.695 s |
| 1024 / 4 | 1,022 / 1,026 | 29,580 | 8.877 s |

Rerun this profile (with a new output directory):

```sh
.deps/rosbag-venv/bin/python scripts/validate_recorded_datasets.py \
  --target dataset/dtvi_aprilgrid.resolved.yaml \
  --quad-decimate-512 2 --quad-decimate-1024 4 \
  --output dataset/validation/new-decimated-iteration
```

Or run a single detection:

```sh
build/release/calib_imu_camera --dataset dataset/1024 --detect-only \
  --target dataset/dtvi_aprilgrid.resolved.yaml --quad-decimate 4 \
  --output dataset/detection-1024-decimated
```

Corners remain in original-image pixels, with refinement on the original image.
The synthetic historical-border check passes for four rotations and decimations
1/2/4, with per-axis error < 1 px on these fixtures. Small-dimension (< 4 px) and
upper-bound (32,768 px) tests guarantee a controlled return instead of an invalid
access/assertion in AprilTag.

For real observations shared by the decimation-1 reference and the profile above,
the corner-difference norm is:

| Resolution | Shared tag observations | Median | P95 | Maximum |
| --- | ---: | ---: | ---: | ---: |
| 512 | 2,778 | 0.040 px | 0.890 px | 6.114 px |
| 1024 | 99 | 0.076 px | 1.137 px | 5.878 px |

Comparing the 11,883 tag observations shared across resolutions (after dividing
1024 coordinates by 2), the difference in 512 pixels is 0.064 px median, 1.367 px
P95, and 11.272 px maximum. Association uses camera, integer timestamp, and ID,
then compares corners with matching indices; tags missing on one side are excluded
instead of being turned into zero errors. Local results: `comparison.json` beside
the second regression.

These are consistency measurements, **not errors against ground truth**. They
expose outliers and do not justify accepting every corner. Robust selection, grid
consistency, reprojection, and observability criteria remain to be built with the
solver.
