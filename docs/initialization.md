# Optical initialization — October 17, 2026

## Input and command

The existing commands' `--initialize-only` mode reads the native dataset, detects
the target, and estimates each camera separately. `calib_imu_camera` additionally
checks the IMU CSV but does not use the IMU for this optical estimate.

```sh
build/release/calib_cameras --dataset dataset/1024 \
  --target dataset/dtvi_aprilgrid.resolved.yaml \
  --initialize-only --models pinhole-equi --quad-decimate 4 \
  --output dataset/initialization-1024
```

The output directory must be new. `--models` accepts `pinhole-radtan` or
`pinhole-equi`: one common value, or one per selected camera in explicit order.
`--init-max-views` defaults to 60 (10 to 200). Normal thread, image-memory, and
camera-selection options apply. `--check-dataset`, `--detect-only`, and
`--initialize-only` are mutually exclusive.

This mode assumes global shutter. It rejects a `--cam` camchain instead of ignoring
its intrinsics or readout; initialization from an existing calibration is a future
extension. No line delay is estimated.

## Method and technical decision

Problem: initialize poses and eight optical parameters without a supplied initial
estimate, including for the fisheye cameras in the private recordings.
Choice: a C++ adapter to **stable OpenCV 5.0.0**, with locked source and hash and
inspected Apache-2.0 license. The `calib` module and its dependencies are built
locally. Alternative: develop a dedicated geometric initialization with
Eigen/Basalt. Reuse avoids immediately reimplementing a monocular calibrator;
future rig/VI/RS residuals remain independent. Synthetic recovery tests and real
regressions may invalidate this choice. No third-party code is copied into our
modules.

1. Divide each camera's frame indices into at most N intervals. In each interval,
   keep the view with the most corners, at least twelve; on ties, keep the first.
   All raw detections remain exported.
2. Reserve every fifth selected view for validation. Require at least ten selected
   views: a minimum of eight fitting and two validation views.
3. Radtan: OpenCV planar initialization with k3 fixed to zero. Equidistant: three
   initial focal lengths (0.3 / 0.5 / 0.8 times the largest dimension), skew fixed
   to zero, pose recomputation, and conditioning checks. Keep the minimum cost
   among trials whose projections are valid and whose distortion Jacobian is
   positive over the sampled domain.
4. Recompute residuals with **our optical model**. Reject corners beyond
   `max(3 px, median + 4.5 × 1.4826 × MAD)`, then estimate once more if necessary.
   Remove a view with fewer than twelve remaining corners, recording per-point reasons.
5. Estimate only the pose of validation views with fixed intrinsics, then evaluate
   **all** their corners without opportunistic filtering on those views.
6. Check positive depth, finiteness, tilt diversity, and residuals. Sample
   distortion over 65 radii × 32 azimuths up to the maximum observed normalized
   radius: require a positive Jacobian and projection→inversion return to the same
   radius, with normalized error ≤ `1e-7 × (1 + radius)`.

Optimization and our verification use double precision. OpenCV's radtan API still
requires float input correspondences; this conversion is confined to the adapter
and exercised by synthetic tests. Stored and exported correspondences remain double.

Cameras initialize in parallel within the worker budget retained after detection;
OpenCV itself is limited to one thread. Images remain processed in bounded batches
and selected views are capped. The CLI memory estimate is an estimated image-space
budget: it does not cap global RSS or internal solver allocations. Cancellation is
checked between OpenCV calls, not during an internal iteration. The signal shared
between workers uses a lock-free atomic boolean.

## Outputs and conventions

- Detection and correspondence files remain available and unchanged.
- `initial-camchain.yaml`: `camera_model`, `distortion_model`, `intrinsics`,
  `distortion_coeffs`, and `resolution`. No fabricated `T_cn_cnm1`, `T_cam_imu`,
  timeshift, or `line_delay`. These are **initial estimates**.
- `initialization.json`: parameters, int64 timestamps, `T_cam_target` poses, point
  count and RMS per pose, identified validation views, rejections with initial
  residuals, failed trials, observed angular domain, and numerical checks.
- `summary.json` reports completed initialization; rig/IMU calibration remains
  `not_performed`. On failure, the command returns nonzero and writes `error.json`,
  without a substitute camchain. Artifacts alone do not replace a successful summary.

`p_cam = T_cam_target * p_target`, with translation in meters. Homogeneous 4×4
matrices are used; this stage exports no quaternion or pose covariance. The target
provides scale. RMS is
`sqrt(sum(||predicted_pixel − observed_pixel||²) / corner_count)`.
Unavailable covariance is `null`.

## Provisional thresholds and limitations

Policy version 1 requires at least 12 corners/view, 10 selected views, and 8 fitting
views after rejection; maximum separation between target normals ≥ 0.08 rad; fitting
and validation RMS ≤ 5 pixels. These safeguards separate static/unusable synthetic
cases from exercised examples. They are not universal quality thresholds or a
marginalized-rank test.

The inversion check samples the **observed** field; it is not a global injectivity
proof or a guarantee out to image corners. Points behind the camera plane remain
excluded. The maximum observed field is not a measurement of the lens's complete
field of view. Reserved views fit their pose from their own corners and come from
the same sequence: their RMS checks intrinsics, not an independent acquisition.

Initial filtering does not replace future robust joint adjustment. It may fail if
most corners are wrong. Low RMS certifies neither extrinsics, uncertainty, nor the
physical sensor model.

## Reproducible tests

`tests/initialization.py` independently generates points with Python formulas,
without the OpenCV renderer: 30 views per model, with known poses and parameters.
It checks noiseless recovery (intrinsics within 0.002 px, coefficients within
0.0003, pose elements within 1e-5), then adds 0.12 px noise per coordinate and
corners shifted by (20,−15) px. Tests also cover a static dataset, insufficient
views, unknown IDs, duplicate timestamps, cancellation, and YAML export reread by
our importer and an independent array reader. The C++ selection test checks the
view-count memory bound, temporal choices, and nanosecond preservation. CLI tests
check incompatible modes, model errors, invalid budgets, and failures without a
fake camchain.

Reproducible private regression, after extraction:

```sh
python3 scripts/validate_initialization.py \
  --output dataset/validation/new-initialization
```

The script checks one/four-worker results for both resolutions, and detections/
correspondences against the previous reference. The complete pixel and IMU audit
from bags remains in `validate_recorded_datasets.py`.

## Executed results

All nine CTest suites pass in C++26 release, with the explicit C++23 fallback and
under ASan/UBSan (the release OpenCV backend is not instrumented). The
`--sources-only` installation was replayed: both archive hashes verified, local
build reused, and installation completed without administrative privileges.
The private `initialization-v1` protocol passes: all five result files are
byte-for-byte identical between one and four workers, while detections, geometry,
and correspondences remain identical to `camera-models-v1`. Each camera uses 48
fitting and 12 validation views. The complete `initialization-inputs-v1` audit also
passes: every pixel and IMU sample matches the source bags; 85,776 / 118,320
correspondences are accepted for 512 / 1024, unchanged from the reference.

| Data | Camera | fx / fy (px) | Fitting / validation RMS (px) | Fitting corners rejected |
| --- | --- | --- | --- | ---: |
| 512 | cam0 | 190.541 / 190.512 | 0.410 / 0.471 | 44 |
| 512 | cam1 | 190.240 / 190.237 | 0.379 / 0.432 | 30 |
| 1024 | cam0 | 381.446 / 381.389 | 0.609 / 1.182 | 116 |
| 1024 | cam1 | 380.282 / 380.296 | 0.663 / 1.452 | 167 |

After dividing the 1024 intrinsics by two, the maximum focal-length difference is
0.183 pixels at 512 scale. Principal-point coordinates differ by at most 0.112
pixels under this simple comparison; resampling may also introduce a pixel-center
convention. Both datasets represent the same acquisition: this consistency does
not replace ground truth or a second acquisition. Projection/inversion checks have
a maximum normalized-radius error of `1.69e-14` over sampled fields.

The larger fitting/validation gap at 1024 must remain in future analysis of
outlier corners and the model; these parameters must not be described as a final
calibration. Failed initial-focal-length trials are recorded.

Observed detection + initialization times: 512, 5.75 s with four workers and
15.25 s with one; 1024, 10.31 s and 36.05 s. Observed RSS: about 21–29 MiB.
These runs shared the machine with other checks/builds: informative measurements,
not an isolated speed benchmark. No solver validation on actual 4K images or a
rolling-shutter acquisition is claimed.
