# Rolling shutter: camera–IMU interface and estimation

Status: detailed design, not implemented. The user has validated the names
`calib_cameras` and `calib_imu_camera`. The model and options below define the
proposed development; no numerical result has been validated yet.

## 1. Reuse the camchain without an additional sensor file

The intrinsic result stores the RS parameter in the corresponding camera entry of
`camchain.yaml`. The canonical Kalibr key is **`line_delay`**, in seconds per row.
Console display and APIs also use LineDelay/lineDelay; those are not the YAML key
written by the inspected official code.

Illustrative excerpt to complete with camera optical parameters:

```yaml
cam0:
  camera_model: pinhole
  resolution: [3840, 2160]
  line_delay: 5.0e-6  # seconds per row, i.e. 5 microseconds
```

The actual camchain also contains intrinsics, distortion_model, distortion_coeffs,
and inter-camera extrinsics when several cameras are calibrated.

```sh
calib_imu_camera --dataset ./session --cam camchain.yaml --imu imu.yaml --target aprilgrid.yaml
```

A nonzero `line_delay` automatically activates the RS model for that camera. The
delay is **fixed by default** during VI calibration; extrinsics, offsets, and other
allowed VI parameters remain estimated. No extra `--shutter` or `--sensor-config`
is needed for this import.

`line_delay: 0` describes the GS limit. Without the field or an RS declaration,
behavior remains GS and is announced. The reader may accept `lineDelay` as an
explicit alias for external files, normalizing it to `line_delay` and rejecting
conflicting values. Only finite scalars are accepted. Preserve the sign of an
imported delay: it determines the vertical temporal-gradient direction and must
not be lost when converting to a positive duration.

Proposed advanced option: `--refine-line-delay cam0`, using the imported value as
initialization before jointly re-estimating it. Bounds are declared through expert
options or a preset. This is independent of `--camera-parameters refine`, which
controls optical/inter-camera geometry. Normal operation requires neither readout
bounds nor a refinement option.

For initial estimation with `calib_cameras --models ...`, declare RS using
`--shutter cam0=rolling` and the required temporal-estimation settings. The result
then retains the value and convention for subsequent commands. Importing `--cam`
therefore reuses a measurement; the field does not remove the need for an
informative sequence when initially estimating the delay.

Readout duration remains derived: for H rows with uniform vertical readout,
`readout_s = (H - 1) * abs(line_delay)`. Example: H=2160 and 5 µs/row gives
10.795 ms between extreme rows. Do not optimize both `line_delay` and readout as
independent parameters.

For special cases (horizontal readout, different time reference, crop, or
rotation), store additional metadata in an optional `rolling_shutter` block in the
same camera entry, specific to our extended format. Its schema and export
compatibility must be validated. No separate sensors.yaml is planned. A `--config`
preset may group estimation options.

Kalibr import without extensions uses its vertical algebraic convention:

```text
t_corner = t_image + delta_row0 + y * line_delay
```

It references y=0. The code does not establish whether the hardware timestamp is
physically the start or midpoint of exposure. With constant exposure, a constant
reference difference can be absorbed into the estimated VI offset, which remains
an effective offset. With unknown variable exposure, a constant offset may no
longer suffice. Do not apply an exposure correction on top of an offset that
already includes it.

Reuse conditions: the same sensor readout mode and the same image geometry, or an
explicitly known transform. Never blindly transfer a 1080p value to a 4K mode or
to a cropped/flipped image.

Verified primary sources:
[Kalibr RS export](https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_rs_camera_calibration/RsCalibrator.py),
[YAML key](https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py),
[temporal model and unit](https://github.com/ethz-asl/kalibr/blob/master/aslam_cv/aslam_cameras/include/aslam/cameras/RollingShutter.hpp).

## 2. Each corner's time belongs to the measurement model

For an explicitly centered convention, distinct from default Kalibr import, with
top-to-bottom vertical readout and a timestamp at the central exposure midpoint:

```text
t_corner = t_png + delta_camera + (y_sensor / (H_sensor - 1) - 0.5) * readout
```

`delta_camera` shifts the entire image in time; `readout` shifts its rows relative
to one another. Readout is defined between exposure midpoints of the first and last
rows. For 12 ms, extreme rows lie at −6 ms and +6 ms around the central reference.

The general equation is given in the [architecture](architecture.md), with rank in
readout order, reference row, and start/mid-exposure correction. If internally
recentered on the middle row, convert exactly
`delta_center = delta_row0 + (H - 1) * line_delay / 2`, then reverse this conversion
on export. Do not change the sign of `timeshift_cam_imu`.
The measured corner pixel determines its visual-residual time. A more rigorous
covariance treatment must account for row error also affecting projection time;
do not present simplified local covariance as absolute metrological truth.

Frame rate does not determine readout. An image reduced to accelerate detection
does not change original-row time. Crop, rotation, and readout mode must be
described; algorithmic detection decimation is not a sensor-mode change.

Timestamp reference cannot be inferred independently of a free offset: changing
the reference row can be compensated by changing the offset. Kalibr import applies
and records its known convention; other models must declare theirs. If hardware
does not confirm that convention, the offset is effective within it, not an
identified measurement of electronic latency; the report must say so.

The initial model assumes linear scanning and constant readout during a sequence.
Exclude or report multi-exposure HDR, electronic stabilization, software warps,
and incompatible mode changes. The model uses exposure midpoint and does not
simulate light integration for a blurred corner. Unknown variable exposure in
PNGs will not be freely estimated to compensate timing errors.

## 3. Joint camera–IMU problem

The metric target is fixed; the camera/IMU rig moves rigidly. Continuous trajectory
`T_W_I(t)` represents the IMU frame in the target world. An SO(3) + R³ spline
provides pose, angular velocity, and acceleration at every usable time.

For every known corner `p_W`:

```text
pixel_predicted = project(K_camera, T_C_I * inverse(T_W_I(t_corner)) * p_W)
```

For every IMU measurement at its original IMU time:

```text
gyro_predicted  = angular_velocity_body(t) + gyro_bias(t)
accel_predicted = R_W_I(t)^T * (p_W_I''(t) - gravity_W) + accel_bias(t)
```

The IMU measures specific force here. The trajectory is at the IMU origin; the
camera lever arm appears in `T_C_I` and the trajectory relationship. Do not confuse
camera acceleration with acceleration measured at the IMU.

Ceres jointly minimizes robustified visual residuals, whitened gyroscope/
accelerometer residuals, and bias constraints. Free parameters are trajectory,
camera–IMU transform, biases, gravity direction, and requested offsets/readouts.
Gravity norm is fixed to a declared value; IMU scale/non-orthogonality is initially
fixed. Camera intrinsics and inter-camera calibration follow `fixed/refine`.

With `--cam` and fixed inter-camera calibration, estimate one anchor `T_C0_I`, then
compose `T_Cj_I = T_Cj_C0 * T_C0_I`; do not release every extrinsic independently
while claiming to preserve the chain. With a free chain, use a minimal set of
transforms and derive the others without redundant parameters.

There is no definitive PNG pre-correction followed by a GS solver. RS projection
depends on motion and the parameters being estimated, so it is reevaluated during
the solve. Original images and detections remain reusable from cache when temporal
parameters change.

## 4. Initialization followed by joint refinement

1. Check timestamps, visibility, coverage, excitation, and usable intervals.
2. Import intrinsics or initialize them from sharp views with little RS distortion.
   A temporary GS approximation is used for initialization only.
3. Initialize trajectory, extrinsic rotation, and offset, notably from consistency
   between visual rotations and gyroscope measurements.
4. Fit a first problem with readout fixed to a plausible hypothesis; if unknown,
   inspect several bounded hypotheses. Strong RS may make one GS initialization
   insufficient.
5. Release requested temporal parameters, then jointly refine. Keep imported delay
   fixed unless `--refine-line-delay` is explicitly requested. Select spline
   resolution through sensitivity validation.
6. Check marginal parameters, stability, residuals, and reserved sequences;
   publish failures and ambiguities alongside results.

Optical calibration obtained while ignoring strong RS must not be considered
reliable by default. Recommend `--camera-parameters refine` or a new optical
acquisition when needed; never silently release parameters the user requested fixed.

Camera-only mode uses the same RS residual and relative continuous trajectory,
without IMU residuals. Fix the reference camera and its offset to remove gauges.
Equal timestamps across cameras do not imply equal corner times. A stationary rig
with a moving target is usable in this mode, but not for VI.

## 5. Derivatives and scientific validation

Basalt's Ceres helper does not itself guarantee temporal derivatives; see
[engine choice](technical-choices.md). Implement and test the derivative chain to
offset/readout, temporal bounds, and spline-support changes. Never round to an
integer while evaluating optimized time.

Independent synthetic cases: known, zero, and strong readout; opposite-sign
offsets; readout directions; GS/RS mixtures; nontrivial transforms and lever arm;
noise; static motion, constant rotation, and varied dynamics. Check rotation,
translation, readout, and offset separately—not only RMS. Test parameter recovery
and uncertainty coverage.

For real data: varied rotations about several axes, accelerations/translations,
corners spread vertically/horizontally, and sufficiently short exposure. Static
views may inform optics but cannot alone measure readout. Repetitive or overly
simple motion may correlate offset and extrinsics.

The report provides readout/offset and uncertainties, correlations with extrinsics,
residuals versus row and time, excitation, proximity to bounds, and stability on
independent windows/sequences. A reserved-data metric also compares GS and RS when
relevant; reduced training cost alone does not prove the usefulness of an extra
parameter.

When information is insufficient: return insufficient/ambiguous status and precise
recommendations, never a hidden arbitrary fixed value. Preserve `line_delay` and
any direction/reference extensions in the camchain and native output. Kalibr
export is partial and explicitly marked if the consumer ignores RS; it does not
reconstruct projections from our complete model.

## 6. Position relative to Kalibr

The [official Kalibr RS documentation](https://github.com/ethz-asl/kalibr/wiki/Rolling-Shutter-Camera-calibration)
describes a monocular tool without simultaneous IMU calibration. Our VI/RS feature
is therefore an in-house estimation development, not a wrapper around that module.

The principle of continuous RS/IMU target-based estimation is documented in
[Continuous-Time Spatiotemporal Calibration of a Rolling Shutter Camera-IMU System](https://arxiv.org/abs/2108.07200).
This reference supports the methodological direction, not equivalent accuracy of
our future code. Kalibr GS will provide a comparison in the zero-readout limit;
the RS case requires synthetic ground truth and suitable real acquisitions.
