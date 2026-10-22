# Implementation roadmap

## Current status

- First C++26 increment: `calib_cameras` and `calib_imu_camera`, with explicit
  `--check-dataset`, `--detect-only`, and `--initialize-only` modes. By default,
  `calib_cameras` now performs synchronized GS rig calibration and generates a PDF.
  `calib_imu_camera` has a first monocular GS solver, with a continuous trajectory,
  extrinsics, time offset, gravity, and constant or piecewise-linear biases under a
  random-walk prior. Rolling shutter remains unavailable; see
  [camera-imu.md](camera-imu.md).
- GS VI: 21 release suites pass, including synthetic recovery of
  extrinsics/biases and positive/negative offsets, Jacobians, and degeneracies.
  Both Oxford paths (`--models` and `--cam`) converge to the same provisional
  estimate: +2.075 ms offset and 0.380 px median held-out reprojection error.
  A complete October 21 replay on all four logical processors compares
  `--threads 4` with automatic selection via `--threads 0`: 53.39 s versus 56.09 s,
  with identical detections/observations and extrinsics equal to machine precision.
  These single measurements establish reproducibility, not guaranteed throughput.
  The EuRoC `dataset-calib-imu1_1024_16` dataset then converged from detection to
  extrinsics in 52.86 s with four workers. The difference from its supplied
  camchain is 1.565 mm and 0.02256°, but the local covariance is deliberately
  unavailable: the nuisance block is singular because of target coupling. This
  case must be diagnosed before making any uncertainty claim for this sequence.
  The VI report now has twelve pages following Kalibr's diagnostic structure:
  3D IMU trajectory and pose axes, six motion components, IMU rate,
  inertial measurements/predictions/residuals, estimated biases, temporal visual
  x/y residuals, and a colored x/y scatter plot. The 1024 replay plots 10,298 IMU
  samples and 59,376 residuals over 977 frames; it passes Poppler PDF analysis and
  inspection of rasterized pages. VI rendering directly uses Python/Matplotlib
  with `PdfPages` and the `Agg` backend; the C++ core neither links the interpreter
  nor publishes a partial PDF.
  ASan/UBSan: the 16 suites predating the new Python smoke tests pass in C++23/O2,
  in three targeted groups totaling 669.30 s. The modified initialization path
  separately passes `vi_calibration_guards` in 53.87 s.
  The 410,768 KiB measurement belongs to the monolithic campaign predating the
  final covariance implementation. A synthetic campaign of 100 trials converges
  completely after correcting boundary initialization, but measures only 80%
  coverage at 95% for the offset and 90% in the joint chi-square test. A grid of
  31 optical perturbations shows up to 120 mm of translation error despite a
  0.661 px held-out median. An HC0 block sandwich estimator was also implemented
  and compared on the same 100 trials: it retains 80% offset coverage and degrades
  the joint chi-square result from 90% to 83%. It remains explicitly experimental
  and is never the default covariance. On Oxford it is correctly unavailable
  because the nuisance parameters require the regularized limit; the central
  estimate remains unchanged within 1.39e-17.
  A temporal block-weight bootstrap is now available only on explicit request.
  Its 8/8 smoke run exactly restores the central estimate; its coverage remains
  unknown. On the 1024 dataset, variable biases with a 0.1 s interval (518 control
  points) converge in 62.26 s and 373,340 KiB. The 0.2 s replay converges in
  49.37 s and 355,384 KiB; extrinsics change by 2.4e-6 degrees and 0.000145 mm,
  and the offset by -0.0106 µs. Both intervals are stable on this single capture,
  without validating the model universally. Priority: pilot block durations,
  propagate optical uncertainty, then perform independent validation before RS.
- Native `camN/<timestamp_ns>.png` and SI-unit IMU CSV reader, decoded-PNG
  validation, int64 timestamps, camera and IMU timing statistics, and raw
  camera/IMU overlap.
- Pinhole/radtan and pinhole/equidistant optical import; signed `line_delay` and
  alias parsing, with resolution checks. Extrinsics are not interpreted.
- Official AprilTag 3.4.5 pinned: native tag36h11/tagStandard41h12 families,
  parallel per-image execution, estimated simultaneous-image budget, and
  deterministic JSONL output with rejections. Tested on mono8/mono16/RGB8 PNG,
  up to 4K.
- Build available with GCC 15.2, CMake 4.2.3, and Ninja 1.13.2 after installation.
  See [validation.md](validation.md) for the checks actually run.
- Deployable installation is available through `scripts/install.sh`: dependencies,
  release build, CTest, and relocatable installation under `~/.local` or an
  absolute prefix. The CMake `Runtime` component contains only the two CLIs, the
  Matplotlib report engine, and two target examples. A clean temporary installation
  was run on October 22, 2026: valid `--version` commands, 18 MiB on disk, and no
  installed AprilTag header or archive. Next deployment step: qualify the script
  on a supported Ubuntu machine without a pre-existing `.deps`, then inventory
  its dynamic libraries.
- Foundation and vision are **partially delivered**: there is not yet an
  application YAML preset, cache, or resume support. A first real camera–IMU
  excitation diagnostic is recorded for Oxford. The GS VI solver now exports the
  marginalized local covariance of the extrinsics and offset, together with the
  target numerical rank; their statistical coverage and global observability
  remain to be qualified.
- A separate ROS1 converter was added at the user's request: it extracts the
  512/1024 bags into `dataset/`, preserves 16-bit PNG, and performs pixel/IMU
  audits and a real-data regression. The C++ reader remains native; MCAP and
  acquisition were not added.
- A historical two-cell tag36h11 border was added after the native mode failed on
  real data; rotation/corner tests and the explicit setting live in a derived YAML.
  See [recorded-datasets.md](recorded-datasets.md). Two real-data regression
  profiles passed (decimation 1, then 512/2 and 1024/4), with one-/four-worker
  output comparisons. Corner differences reveal outliers that must be handled
  before considering the calibration reliable.
- The RS path is retained: `line_delay` is fixed by default in VI, refinement is
  explicit, and no `sensors.yaml` is required. The continuous model and four readout
  directions remain to be implemented.
- Metric target geometry and point/pixel correspondences are implemented, with
  stable identifiers, structural rejections, and exports separate from raw
  detections. Conventions and tests are in [observations.md](observations.md).
  Five suites pass under release/ASan/UBSan; the 512/1024 regression with an
  independent export audit passes without changing the reference detections.
- Pinhole-radtan and equidistant projections, analytical point/parameter Jacobians,
  and a camchain adapter are implemented in a module without an external optical
  dependency. Seven suites pass under release/ASan/UBSan with the optional OpenCV
  oracle, including 632 finite-difference cases and 640 independent comparisons.
  Both optical tests also pass in C++23. The real `camera-models-v1` regression
  succeeds with unchanged detections and correspondences. Domains and limits are
  explicit in [camera-models.md](camera-models.md).
- Intrinsic and target-pose initialization is delivered for radtan/equidistant:
  pinned OpenCV 5.0.0, bounded views distributed over time, held-out-view checks,
  tracked rejections, and projection/inversion verification over the observed
  field. It exports `initial-camchain.yaml` and `initialization.json`, without
  fabricated extrinsics. Nine suites pass under release/ASan/UBSan and C++23.
  On both private resolutions, results are identical with one/four workers and
  detections are unchanged; scaled intrinsics are consistent, but do not constitute
  ground truth. See [initialization.md](initialization.md) for provisional thresholds
  and limitations.
- First GS camera–camera calibration delivered: automatic graph initialization,
  joint Ceres 2.2.0 refinement, explicit gauge, extrinsic export, held-out views,
  and an initial PDF. Eleven suites pass under release, ASan/UBSan, and C++23;
  the 512/1024 regression passes with one/four threads, unchanged detections, and
  transformations stable within 1e-8. Provisional quality: full rank, marginalized
  covariance, Kalibr import, and an independent real capture still require
  validation. See [camera-camera.md](camera-camera.md) for the protocol and results.

### Next VI increment

1. Compare `0.5`, `1`, and `2` s blocks on a separate pilot, lock this choice,
   then qualify bootstrap coverage on a paired campaign.
2. Propagate intrinsic uncertainty—or provide a profile analysis—instead of
   silently conditioning extrinsics on exact optics.
3. Extend sensitivity analysis for variable biases and spline intervals, then
   repeat validation on an independent capture before any rolling-shutter solver.

## Milestones and expected evidence

| Stage | Usable deliverable | Criterion before declaring the stage complete |
| --- | --- | --- |
| 1. Foundation | Reproducible build, explicit CLI, optional configuration, native PNG/CSV reader, timing checks | Clean build with pinned dependencies; valid/invalid fixtures; tested conventions |
| 2. Vision | AprilGrid, cache, memory-bounded 4K processing | Corners/IDs validated across families and borders; decimation bias measured; cache invalidation |
| 3. GS camera–camera | Intrinsics, extrinsics, export, and PDF from one command | Synthetic ground truth; disconnected graph rejected; Kalibr import and independent real sequence |
| 4. GS camera–IMU | Continuous trajectory, extrinsics, offset, biases | Synthetic recovery with correct offset sign; static/degenerate cases rejected; real-data comparison |
| 5. Rolling shutter | Per-camera models, fixed/estimated readout, and diagnostics | Four directions, GS limit, crop, segment crossings, offset/readout correlation; real RS data |
| 6. Qualification | Marginal uncertainty, validation, recommendations, performance | Statistical coverage over synthetic repetitions; window bootstrap; benchmarks and RSS |
| 7. Later extensions | ROS/MCAP adapters and capture only if requested | Timestamps and frames preserved; 4K throughput and losses measured on the target platform |

The report and diagnostics start in stage 3; stage 6 qualifies and expands them.
A stage does not become functional merely because a flag or empty class exists.
Prioritize the formats actually supplied by the user without coupling the solver
to their container.

## Validation campaign

### Synthetic data

Keep the generator independent of the residual implementation as far as possible:
known geometry, poses, intrinsics, noise, biases, offsets, RS readout, and missing
data. Test modeling biases (imperfect target, incorrect timestamps, blur,
saturation) separately from noise that matches the solver assumptions.

Check translation, geodesic rotation, offset, and readout errors against declared
tolerances based on noise and excitation. Do not rely on final cost alone. Compare
Jacobians with finite differences and cover invalid projection domains. Over noisy
repetitions, verify empirical coverage of uncertainty intervals, not just that they
are finite.

### Real data

Select TUM-VI/EuRoC/Kalibr calibration sequences whose target, sensors, and
timestamps are documented. Plan an appropriate RS dataset and captures from the
target rig; 4K images alone do not qualify the RS model.

Compare Kalibr and Basalt, then OpenMVIS if reproducible, with the same observations,
models, assumptions, and windows. Document selection and weighting differences.
A Kalibr output is not ground truth: add repeatability and, when available, an
independent physical rig measurement.

### Performance

Compare 720p, 1080p, and 4K; number of cameras, duration, and corner count. Record
CPU, RAM, storage, compiler, libraries, threads, cold/warm cache, each stage's
duration, and peak RSS. Evaluate speedup and numerical stability across
1/2/4/8/... cores. Include a long sequence to verify bounded image memory and
measure optimization-problem growth separately.

## Choices to clarify with the user

1. CPU architecture, camera count, effective inter-camera synchronization, and overlap.
2. Hardware, optical models, IMU, and availability of hardware/exposure timestamps.
3. Existing or new target; dimensions and expected metrological quality.
4. Target tolerances in mm/degrees/ms, computation time, and available resources.

Established choices: Linux, defined native dataset, SI units, common clock, and
offline calibration. Initial physical assumption: fixed target and rigid rig for
camera–IMU. User decision: no theoretical placement as camera–camera input.
Relative placement is initialized automatically from observations; comparison
with the theoretical rig remains an external user check. Next verifiable increments
for the current cam0 + IMU scope:

1. Correct offset undercoverage without a universal empirical factor: evaluate a
   sandwich/Godambe estimator or bounded bootstrap, then repeat the 100 trials and
   require coverage compatible with the claimed level.
2. Propagate intrinsic uncertainty, complete sensitivity analysis for spline and
   bias intervals, then use an independent real capture. After that, address
   rolling shutter, with `line_delay` fixed by default.
3. Add the observation cache with fingerprints, tested corruption/invalidation,
   and resume support; measure time and memory before and after.

GS camera–camera qualification remains deferred during this camera and IMU
priority: rank after gauge fixing, marginalized covariance, outlier influence,
and independent real-data validation remain to be addressed.

The supplied real datasets (512/1024) were used for historical regressions but
were not present in this environment on October 19, 2026. The new VI increment
uses Oxford cam0 + IMU. None of these datasets provides extrinsic ground truth;
independent captures are still required.

Oxford Spires 2024-03-19 import prepared: sequential multi-bag reading and
camera-only export were tested. The inventory finds 1,699 images with a zero
sensor timestamp, no IMU, and a checkerboard target. At the user's request, wait
for valid sensor timestamps; no native March Oxford dataset was published. Details
and resumption criteria are in [recorded-datasets.md](recorded-datasets.md).

Oxford Spires 2024-10-29: camera–camera and camera–IMU captures were separately
extracted and audited with their sensor timestamps. At the user's request, only
the cam0 + IMU subset is now retained locally. Its AprilGrid detection,
`pinhole-equi` initialization, one-/four-worker determinism, coverage, relative
sharpness, half-sequence stability, and IMU excitation were checked. Initialization
is usable as a seed but remains uncertified; it is neither a camera–IMU extrinsic
nor a temporal estimate. The first VI solver is now exercised on this dataset,
after synthetic recovery of extrinsics/biases and offsets of both signs. Its
marginalized local covariance is computed without loading a global dense matrix.
Synthetic offset coverage is insufficient; the next step is to correct this
diagnostic and perform independent validation. See
[recorded-datasets.md](recorded-datasets.md).
