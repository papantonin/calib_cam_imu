# Proposed architecture

Status: design revised after clarification of the dataset and CLI style. Criteria
for declaring a feature available are listed in the [roadmap](roadmap.md).

## 1. Scope and assumptions

Priority is given to metric camera–camera and camera–IMU extrinsics, their
uncertainty, and acquisition diagnostics. The tool can fix known intrinsics or
estimate them; their error propagates into extrinsics. Camera models must be
explicit—never select a complex model automatically from RMS alone.

Confirmed platform: Linux; x86_64 is proposed first, with hardware architecture
still to be specified. Confirmed input: `camN/<timestamp_ns>.png` and
`imu0/imu0.csv`, CSV fields `timestamp_ns,wx,wy,wz,ax,ay,az`, rad/s and m/s²,
and a common clock with offsets to refine. See the [dataset contract](dataset.md).
Future ROS 2/MCAP and ROS 1 adapters remain separate from the core, with no current
implementation or dependency. Direct capture is outside the first version, as
chosen by the user. Initial 4K support therefore targets reading, detection, and
offline calibration.

For camera–IMU: a rigidly attached moving rig and a target fixed in the world.
A moving target and stationary rig do not provide the required inertial constraints.
For synchronized global-shutter camera–camera calibration, a fixed rig and moving
target are possible. The co-observation graph must connect the cameras; cameras
without overlap require a validated additional model, never a silently applied
simple stereo calibration.

## 2. Technical stack

| Element | Proposal | Rationale and alternative |
| --- | --- | --- |
| Language | C++26 where supported, declared C++23 fallback | Modern language without relying on unimplemented features |
| Build | CMake, Ninja, presets, vcpkg with a locked baseline | Reproducibility; Basalt already provides an overlay |
| Geometry | Evaluate Basalt components, Eigen, Sophus | Targeted and replaceable reuse; Basalt is not mandatory |
| Detection | Official AprilTag 3 + AprilGrid geometric validation | Controllable decimation and parallelism |
| Solver | Ceres, double, manifolds, sparse SuiteSparse algebra | Robust batch estimation; GTSAM remains an alternative to compare |
| Images | OpenCV limited to codecs, conversions, and useful primitives | No GUI or opaque calibration engine in the core |
| Scheduling | oneTBB, global CPU budget | Bounded queues, concurrent stages, concurrency limits |
| CLI/config | CLI11, yaml-cpp, versioned schema | Few arguments, strict reusable configuration |
| Report | Cairo/Pango PDF, JSON metrics | Standalone report without browser or runtime Python |

Dependencies are proposed, not yet installed in this design snapshot. Lock
versions, hashes, and licenses when integrating them; do not claim “latest” without
verification. Minimize optional packaging dependencies. The
[engine comparison](technical-choices.md) covers Ceres with/without Basalt, GTSAM,
and in-house development. Recommendation: an in-house application using Ceres and
targeted reuse of validated geometry components.

## 3. Modules and processing pipeline

```text
input → checks → decoding → detection/cache → informative selection
      → initialization → optimization → validation → YAML/JSON/PDF
```

Planned modules: `io`, `target`, `geometry`, `trajectory`, `solver`, `quality`,
`report`, and `cli`. Timestamped observations and the solution form the module
interfaces; report generation never reruns optimization. See the
[interface contracts](interfaces.md). The CLI provides `calib_cameras` and
`calib_imu_camera` with Kalibr-like arguments; the reader neither selects the
calibration mode nor performs temporal association.

1. Check units, dates, missing images, duplicates, order, resolution, IMU
   saturation, and temporal overlap. Report reception timestamps.
2. Detect tags and assemble target corners while checking family, IDs, corner
   order, geometry, local quality, and neighborhood consistency.
3. Select observations covering positions, distances, tilts, and motions. In
   VI/RS, preserve dense temporal windows and associated IMU data.
4. Initialize intrinsics, target poses, and camera–camera extrinsics. Initialize
   IMU/camera rotation and offset from angular velocities; bound the search and
   inspect competing maxima for periodic motion.
5. Progressively estimate trajectory, biases, gravity, extrinsics, and offset.
   Then release requested RS/intrinsic parameters; an imported `line_delay` remains
   fixed by default in VI unless refinement is explicitly requested.
6. Jointly refine observable parameters. Retain several initializations when
   ambiguity is substantial.
7. Evaluate quality and stability on reserved data and produce all diagnostics,
   including when scientific validation fails.

## 4. Spatial and inertial model

`T_A_B` transforms B into A. World frame W is attached to a fixed metric target;
body frame I is the reference IMU. For a known corner `p_W`:

```text
p_C(t) = T_C_I * inverse(T_W_I(t)) * p_W
z_pred = project(camera_model, p_C(t))
```

In camera-only mode, the body is the reference camera. Independent poses per time
are sufficient for synchronized global shutter; VI, RS, and asynchrony use a
continuous trajectory. A Basalt SO(3) + R³ spline is the starting point. Its knot
spacing must match the dynamics and be checked through validation: too coarse
biases the result, too fine absorbs calibration errors.

At the IMU origin, the ideal model is:

```text
gyro(t) = angular_velocity_body(t) + bias_g(t) + noise_g
accel(t) = R_W_I(t)^T * (p_W_I''(t) - gravity_W) + bias_a(t) + noise_a
```

Residuals are whitened by their noise; bias random walks and discrete covariances
are computed from the spectral-density convention and actual time step. Noise
parameters come from characterization or an explicit profile, not a universal
value. A future Allan command on a long static acquisition may help obtain them.
IMU scale/non-orthogonality remains fixed initially; releasing all such parameters
without sufficient excitation makes estimation fragile.

Visual observations use robust losses and traceable rejection decisions. Use
analytical Basalt Jacobians when available and compatible; use Ceres autodiff for
other blocks, with independent numerical tests.

Camera–camera uses Schur elimination of target poses. Continuous trajectories
couple neighboring knots; do not impose the same elimination order. Compare sparse
Cholesky and suitable ordering using measured memory and duration.

## 5. Rolling shutter

The [RS/IMU specification](rolling-shutter.md) details the command, initialization,
joint parameters, degenerate cases, and required validation.

Current input is `line_delay` in the camchain, in seconds per row. Kalibr import
applies `t_corner = t_image + offset_row0 + y * line_delay`, activates RS for a
nonzero value, and reuses this parameter fixed by default in VI. No separate sensor
file is needed. For H rows, readout between extreme rows is
`(H - 1) * abs(line_delay)`; preserve the sign for readout direction. Any additional
conventions remain in the same file.

Each camera owns its shutter model. Parameters are offset, readout direction,
positive readout duration, timestamp reference, exposure, sensor dimensions, and
the transform between stored-image and sensor pixels.

Convention: `readout_s` is the interval between exposure midpoints of the first
and last read rows. If `s` is the continuous rank in readout order and N is the
number of rows (or columns for horizontal readout):

```text
q = s / (N - 1)
t_corner = t_stamp + offset + (q - q_ref) * readout_s + exposure_correction
```

Row centers run from 0 to N−1. `q_ref` is 0, 0.5, or 1 according to the timestamp's
reference row; `exposure_correction` is 0 for exposure midpoint and +exposure/2
for exposure start. The profile must remove ambiguity. Per-row duration is
`readout_s/(N-1)` under this convention, not `readout_s/N`. Reject N below 2 in
RS mode; zero readout recovers global shutter.

Corners remain expressed in original sensor rows after decimation, rotation, or
cropping. Without sufficient metadata, do not invent their time. Options include
global/rolling per camera, fixed/estimated readout, bounds, optional prior, one of
four directions, and fixed/estimated offset. Direction is an explicit discrete
choice; an optional search compares several hypotheses.

For Ceres, first inspect Basalt's `ceres_spline_helper.h`. In the inspected
revision, normalized time is a `double`, so the helper does not guarantee offset
autodiff. Integer-timestamp APIs also require adaptation: do not round the optimized
offset. Provide a continuous time derivative and manage spline support across the
entire permitted offset/readout interval. Test segment transitions; never retain
incorrect knot blocks after a temporal displacement.

Low angular variation cannot properly distinguish readout, offset, and pose.
Report their correlation; do not claim a reliable estimate merely because the
solver terminated. The exposure-midpoint model does not itself correct motion
blur: measure it and recommend shorter exposure.

## 6. CPU performance and 4K

- Progressive reading, parallel decoding, reusable buffers, and bounded queues.
  A 3840×2160 mono8 image uses about 8.3 MB; RGB8 about 24.9 MB excluding
  allocations. Two 30 Hz mono8 streams represent roughly 498 MB/s raw.
- Fast detection may use internal decimation, with corners refined at full
  resolution. Validate bias and repeatability against non-decimated detection.
  Rerun at full resolution for small tags.
- No systematic downscaling to 1080p or final coordinates at low resolution.
- One detector per worker. When frames run in parallel, limit internal AprilTag/
  OpenCV threads; give the solver phase the available CPU budget. Do not let BLAS
  multiply threads again.
- Cache corners and metadata using a versioned index and input checksums. Preserve
  timestamps, options, target, resolution, and detector version.
- Release images from memory after extraction. The budget must also cover
  observations, Jacobians, and factorization: estimate cost before the solver and
  reject explicitly if the budget cannot be met.
- Measure reading/decoding/detection/solver/report separately, peak RSS, and
  scaling across 1/2/4/8/... cores. Make no speed promise without a dataset and machine.
- Reproducible mode: record stable order, seed, and thread count; declare numerical
  tolerance without promising bitwise identity across architectures.

## 7. Quality, uncertainty, and report

A global `pass`, `warn`, or `fail` summarizes explicit versioned criteria.
Uncomputable metrics are `unavailable` with a reason. Numerical solver success and
scientific status remain distinct fields.

| Family | Measurements | Useful operator feedback |
| --- | --- | --- |
| Inputs | Duration, frequency, jitter, gaps, duplicates, saturation | Fix timestamping, transport, or IMU range |
| Vision | Corners, spatial coverage, distance, tilt, blur | Show borders; vary distance and angles |
| Reprojection | RMS, median, P95, maps, and residuals versus time/row | Inspect optics, blur, RS, synchronization |
| IMU | Raw/whitened residuals, biases, autocorrelation | Revisit noise, vibration, or model |
| Excitation | Rotation-axis and acceleration diversity | Add rotations and translations on missing axes |
| Identifiability | Rank, weak modes, correlations | Record an exciting sequence or fix a known parameter |
| Generalization | Reserved windows, cross-sequence stability | Acquire an independent sequence |
| Metrology | Translation/rotation/time/readout uncertainties | Compare with application tolerances |

Normalize Jacobian columns before interpreting conditioning: meters, radians, and
seconds are not directly comparable. Fix gauges, then eliminate trajectory/bias/
other nuisances to estimate marginal extrinsic information. Document fixed
intrinsics: resulting uncertainty is conditional and may be optimistic.

Provide local covariance in tangent space with a declared convention; it assumes
an adequate model and noise. In thorough mode, complement it with temporal-window
bootstrap and repeatability across acquisitions. Incorrect target size or imperfect
planarity can create systematic bias invisible to this covariance.

Reserve temporal windows before selection and rejection. For a validation window,
estimate only the required nuisance variables while holding calibration fixed and
state this protocol. Do not present residuals on the same fitted corners as
independent validation.

The PDF contains verdict and limitations, provenance/configuration, frame diagram,
extrinsics and uncertainties, camera/time/RS parameters, coverage and poses,
residuals, excitation, stability, rejected data, and acquisition recommendations.
Pixel thresholds are not universal, especially between 720p and 4K: also publish
normalized residuals and tie acceptance to application tolerances.

## 8. Exports

The native result is versioned and includes units, frames, and status. Kalibr
export uses `T_cam_imu = T_C_I`, `T_cn_cnm1 = T_Cn_Cn-1`, and
`t_imu = t_cam + timeshift_cam_imu`. Test import and reprojection with an independent
consumer, including intrinsics and distortion ordering.

Kalibr exports `line_delay` for its RS model; preserve this key and convention in
the camchain. The field's presence does not mean its VI solver can use it.
Unrepresentable extensions remain in the native format and a documented extended
camchain block, without a mandatory sidecar. Report data loss when a strict export
cannot represent them. Never silently transform an optical model merely to produce YAML.
