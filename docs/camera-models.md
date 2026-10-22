# Optical models — October 17, 2026

## Scope

`calib_camera_models` performs optical calculations in double precision, with no
allocation per projection, external dependency, or access to formats or the CLI.
`make_camera_model` adapts an imported camchain to this typed interface and
validates its eight parameters. This prepares reprojection residuals; no parameter
is estimated at this stage. User commands remain unchanged.

Choice: local analytical formulas and derivatives, with an OpenCV comparison as
an independent oracle. The alternative is to immediately integrate a broader
model library, especially Basalt. The current scope needs only two models; the
narrow interface permits later substitution. A discrepancy with independent
values or finite differences would invalidate this implementation. No OpenCV or
Kalibr code was copied.

## Conventions

The input is a point `p_cam = T_cam_target * p_target`, in meters, in the camera
frame: X right, Y down, Z forward. Output `(u,v)` is in pixels at original
resolution, without rounding, cropping, or clipping. Skew is zero.

| Camchain model | Parameters and order |
| --- | --- |
| pinhole / radtan | `fx,fy,cx,cy,k1,k2,p1,p2` |
| pinhole / equidistant | `fx,fy,cx,cy,k1,k2,k3,k4` |

With `x=X/Z`, `y=Y/Z`, and `r²=x²+y²`, radtan applies:

```text
s = 1 + k1*r² + k2*r⁴
xd = x*s + 2*p1*x*y + p2*(r² + 2*x²)
yd = y*s + p1*(r² + 2*y²) + 2*p2*x*y
```

Equidistant uses `theta=atan(r)`, then
`theta_d=theta*(1+k1*theta²+k2*theta⁴+k3*theta⁶+k4*theta⁸)` and
`(xd,yd)=(x,y)*theta_d/r`. The on-axis limit is evaluated analytically; a series
for `atan(r)/r` and its derivative avoids cancellation for `r²<1e-8`.
Both models produce `u=fx*xd+cx`, `v=fy*yd+cy`.
Zero equidistant coefficients **do not produce a perspective camera**.

Each projection returns two analytical Jacobians: `2×3` with respect to `(X,Y,Z)`
and `2×8` with respect to the parameters above, with `(u,v)` rows. These are not
SE(3) pose derivatives or covariances; no pose perturbation convention is
introduced here.

## Explicit domains and limitations

- Strictly positive focal lengths; finite parameters and points.
- `Z>0` for both models. The exact horizon and rear hemisphere are rejected:
  fisheye support beyond a 180° field of view is not provided.
- Pixels outside the image are returned so a future solver can compute residuals;
  they do not become valid observations.
- Numerical overflow in pixels **or derivatives** produces a typed error, never a
  substitute value. Extreme directions whose normalization overflows may therefore
  be rejected even if another parameterization could evaluate them.
- A finite polynomial evaluation does not prove that distortion is injective.
  This module provides no global fold-over check, pixel-to-ray inversion, usable
  field-of-view estimate, or physical-validity diagnostic.
- `line_delay` remains imported separately. Corner time and trajectory determine
  the camera point before projection; this module does not provide rolling-shutter
  calibration.

## Verification

`camera_model_tests` checks simple analytical values (perspective, radtan,
equidistant), the optical axis, both sides of the numerical transition, quadrants,
large angles up to 89°, scale invariance, and domain errors. Across 632 cases, all
eleven Jacobian columns are compared with centered finite differences: absolute
tolerance `2e-6`, relative tolerance `2e-7`. The camchain adapter is also tested
for both parameter orders.

`camera_model_oracle.py` compares 640 reproducible cases against
`cv2.projectPoints` and `cv2.fisheye.projectPoints`, including their compiled
Jacobians. Because the pose is the identity, translation derivatives equal the
camera-point derivatives. Pixel tolerances: absolute `2e-9`, relative `2e-13`;
derivatives: absolute `2e-8`, relative `2e-10`. This optional test requires OpenCV
and NumPy; see [INSTALLATION.md](../INSTALLATION.md).

October 17 run: seven CTest suites passed in C++26 release and under ASan/UBSan,
with the optional oracle enabled; both optical suites also passed with the explicit
C++23 fallback. With OpenCV 4.10.0, the maximum projection difference was
`3.64e-11` pixels. Syntax, help, and the installer's `--sources-only` path were
verified; the optional apt installation was not rerun because its modules were
already present.

Real-data regression: `dataset/validation/camera-models-v1/regression.json`, status
`passed`, with a complete pixel/IMU audit and detection using one/four workers on
the 512 and 1024 datasets. Compared with `metric-observations-v1`, detections,
geometry, and correspondences are byte-for-byte identical. This demonstrates no
regression in the existing path; no real-data projection is validated because
reference intrinsics/poses are unavailable. Durations from this run, which shared
the machine with builds, are not an isolated benchmark.

These tests qualify the optical calculation in exercised cases, not calibration
accuracy or the validity of a model for a real lens. The next step is initialization
of intrinsics and target poses, followed by robust optimization with independent
validation.
