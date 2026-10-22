# Synchronized camera–camera calibration

First functional stage: global shutter, exactly shared timestamps, a known metric
target, and a rigid rig. No theoretical placement is requested. Pinhole-radtan
and pinhole-equidistant models are available. This stage solves neither camera–IMU,
inter-camera offsets, nor rolling shutter.

## Usage and outputs

```sh
build/release/calib_cameras --dataset dataset/1024 \
  --target dataset/dtvi_aprilgrid.resolved.yaml --models pinhole-equi \
  --quad-decimate 4 --threads 4 --output dataset/calibration-1024
```

The output directory must be new. `--cameras cam0 cam1` selects cameras and their
order; otherwise directories are ordered numerically. The first camera fixes the
frame. `--models` accepts one common model or one value per camera.
`--init-max-views` limits initial views per camera and synchronized solver groups
to 60 by default (10–200). Detection still processes every image.
`--initialize-only` retains the previous monocular-only initialization path.
`--cam` is rejected in both paths so supplied parameters cannot be ignored.

- `initial-camchain.yaml`, `initialization.json`: intermediate monocular results.
- `camchain.yaml`: refined optical parameters and `T_cn_cnm1` matrices.
- `calibration.json`: `T_cam_reference` matrices, optical centers, target poses,
  residuals, excluded views, downweighted corners, convergence, and limitations.
- `report.pdf`: summary, RMS, matrices, and per-camera histograms. This first
  English text/vector report does not yet include coverage maps or IMU plots.
- The usual observation, geometry, summary, and raw-detection exports remain.

Convention: `p_cam = T_cam_reference * p_reference`, translation in meters.
`T_cn_cnm1 = T_cam_reference * inverse(T_previous_reference)` transforms the
previous selected camera into the current one. The optical center in the reference
frame is `-Rᵀ t`, which differs from the matrix translation column. Identity of
the first camera is a declared gauge. The target fixes scale. Numeric fields follow
Kalibr conventions; import into a complete Kalibr run has not yet been tested.
ROS topics are not fabricated.

## Solution and checks

1. Reserve one view in five among uniformly sampled synchronized timestamps.
   These timestamps are excluded from the entire monocular initialization and
   joint optimization. Images without enough points remain in detection outputs;
   unusable groups are listed in rejections.
2. Estimate target poses from initial intrinsics. Every graph edge requires at
   least six shared training poses. Initialize its transforms with a medoid of
   relative estimates, then traverse the graph from the reference. Reject a
   disconnected graph.
3. Jointly refine eight optical parameters per camera, six pose parameters per
   non-reference camera, and six per training target pose. Ceres 2.2.0, automatic
   differentiation, sparse Schur/Eigen Sparse, 2 px Huber loss. Large residuals
   are downweighted and retained in raw statistics. Images, initialization, and
   solver successively use the same CPU budget.
4. On every reserved view, fix calibration and estimate the target pose from
   **one anchor camera only** (first available in selected order). Reprojection
   into the other cameras checks inter-camera consistency. Anchor-camera RMS is a
   less independent check because the target pose uses its own corners. The anchor
   is recorded for every view.

Version-1 policy thresholds are provisional: at least eight usable training groups
and two validation groups, normal diversity ≥ 0.08 rad, raw RMS for each camera
≤ 5 px in both sets, projections in front of the camera, and sampled
projection/inversion checks over the observed field. They detect gross deficiencies;
they guarantee neither accuracy nor observability. The pixel threshold is not
resolution invariant. A pose failure on a validation view fails calibration; it
is not discarded to improve the score. Reserved views belong to the same acquisition.

Convergence yields `quality: provisional_not_certified`,
`observability: full_rank_not_assessed`, and `covariance: null`. Full rank,
marginalized covariances, spatial coverage, and the influence of outlier corners
remain to be qualified. Initialization and optimization may converge to a local
minimum. A common clock alone does not ensure synchronized exposures; offset
estimation will require a later temporal model.

## Validation

On October 17, 2026, 11/11 suites passed in C++26 release, ASan/UBSan, and C++23
with GCC 15.2 and the Python optical oracle enabled. The instrumented rig suite
takes about 233 s versus 2.3 s in release. OpenCV and Ceres remain uninstrumented
release libraries; instrumentation covers our adapters/residuals and AprilTag.
The `--sources-only` installation path and shell syntax check also succeeded.
One added CLI test first failed on the expected wording (“two cameras” versus
“two selected cameras”); its assertion was corrected and rerun successfully,
without an algorithm change.

The `rig_derivatives` and `rig_calibration` suites respectively check:

- 140 residual configurations and all 20 Jacobian columns against finite
  differences: both models, optical axis, zero and near-π rotations.
- Geometry independently produced in Python: known optics and extrinsics, reversed
  camera order, a three-camera chain with no direct cam0/cam2 overlap, noise,
  outliers, static degeneracy, disconnected graph, inconsistent timestamp, and
  cancellation. Without noise: matrix error < 2e-5 and optical-parameter error
  < 0.01. With noise/outliers: matrix error < 0.003 and validation RMS for cam1
  < 0.5 px in this simulation.
- Independent YAML matrix and PDF reading with Poppler, plus visual inspection of
  one page from the real 1024 report. The test does not install Kalibr.

Reproducible private regression:

```sh
python3 scripts/validate_rig.py --output dataset/validation/rig-v1
```

The script calibrates 512 then 1024 with one/four threads, compares detections to
`camera-models-v1`, checks exclusion of reserved timestamps, rereads PDFs, and
verifies numerical stability: 1e-6 for optical parameters/RMS and 1e-8 for
transform entries. It retains raw measurements and the binary fingerprint in
`regression.json`. Time and RSS are indicative, not a benchmark isolated from
machine load.

Results from the first real run (decimation 512/2 and 1024/4):

| Resolution | Training RMS cam0 / cam1 (px) | Validation RMS cam0 / cam1 (px) | cam1 center in cam0 (mm) |
| --- | --- | --- | --- |
| 512 | 0.560 / 0.590 | 0.622 / 0.608 | 101.011; 1.768; 1.465 |
| 1024 | 1.472 / 1.381 | 1.666 / 1.265 | 101.117; 1.719; 1.236 |

The position difference is about 0.257 mm. Both resolutions come from the **same
acquisition**; this agreement does not measure absolute accuracy. The 1024 run has
more large residuals, which remain visible in the report. A mechanical measurement
or a new acquisition is still required for independent validation. No IMU/RS
result is inferred from these trials.

The `dataset/validation/rig-v1` regression passed: 42 training and 9 validation
groups at 512; 44 and 10 at 1024. Detections/correspondences remain identical to
the reference. Between one/four threads, the maximum observed difference was
2.43e-16 for transform entries, 2.28e-13 for optical parameters, and 3.30e-8 px
for RMS. Small RMS differences notably come from numerically recomputed validation
poses; comparison does not require bitwise-identical solver results.

Indicative complete times: 4.32 / 16.05 s at 512 and 9.70 / 36.26 s at 1024
(four / one threads). Corresponding RSS peaks: 28,412 / 21,580 KiB and
35,900 / 23,844 KiB. These measurements cover 2,076 images per resolution; both
settings have the same view budget and observations. They do not establish
guaranteed throughput at 4K or on another machine.

Next step: rank diagnostics after fixing gauges, extrinsic uncertainty after
marginalizing intrinsics/poses, and qualification over synthetic repetitions
before interpreting those uncertainties on real data.
