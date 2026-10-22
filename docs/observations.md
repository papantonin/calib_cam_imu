# Geometry and correspondences — October 17, 2026

With `--detect-only --target ...`, the command now writes two additional files
without a new option: `target_geometry.json` and `observations.jsonl`.
`detections.jsonl` retains the raw observations and its previous format. The
exploratory `--tag-family` mode does not produce geometry.

## Frame and identifiers

The target is planar, Z=0. The origin is corner p0 of the first tag (`tagStartId`).
X follows columns, Y follows rows, and Z=X×Y. IDs increase by column, then by row;
tags share their canonical orientation. Tag pitch is
`tagSize * (1 + tagSpacing)`. Size means the side between detected border corners,
excluding the outer margin. For tagStandard41h12, this border is not the complete
symbol envelope.

For each tag, corners p0/p1/p2/p3 respectively correspond to
`(0,0)`, `(s,0)`, `(s,s)`, `(0,s)` in its local square, with `s=tagSize` in meters.
This convention is consistent with the geometry documented by
[Kalibr](https://github.com/ethz-asl/kalibr/blob/master/aslam_cv/aslam_cameras_april/src/GridCalibrationTargetAprilgrid.cpp).
Kalibr code is not copied. On real 1024 observations, the direction of neighboring
IDs was also compared with edges p1−p0 and p3−p0: median cosines are 0.999816 and
0.999736, respectively. This checks the convention, not target accuracy or planarity.

Our stable identifier is `point_id = 4*(tag_id-tagStartId)+corner_index`.
It is **not** Kalibr's flattened point index; a Kalibr export must remap it.
The exported geometry explicitly lists every association. The user's 6×6 target,
with 0.088 m tags and 0.3 spacing, contains 144 points and spans 0.66×0.66 m from
the declared values, without independent metrology.

Choice: place the origin at the first tag to simplify comparison with Kalibr
conventions. The alternative is the target center; it changes the pose frame
without adding information. A rotation, shifted-ID, or correspondence test that
contradicts the ordering above must block solver use.

## File contract

`target_geometry.json` contains the frame, units, extent, and a list of
`point_id`, `tag_id`, `corner_index`, and `position_m`.
`observations.jsonl` contains one record per image: camera, unchanged int64
timestamp, original resolution, `point_id`/`pixel` list, rejections, and status.
Consumers join points by identifier, never by array position or image number.
Pixels are not reordered within a tag, resized, or corrected at this stage.
No intrinsics, pose, or offset is required for this export.

A tag is excluded from correspondences if its ID is outside the grid or duplicated,
a corner is non-finite or outside the image, or its quadrilateral is degenerate,
non-convex, or inconsistently ordered. Rejection applies to all four corners and
retains its reason. Raw detections remain available; no AprilTag decision margin
is converted into covariance. The current detector still aborts on duplicate IDs;
the observation builder can also reject them for future imported-observation users.

Incomplete outputs use the `.incomplete` suffix. Geometry alone does not prove
successful processing: inspect `summary.json` or `error.json`. The calibration
report remains `not_performed`; current filters certify neither reprojection,
observability, nor uncertainty.

## Tests and next step

The `observations` suite checks a rectangular grid with a nonzero starting ID,
known metric coordinates, rotations and perspective, out-of-bounds corners,
invalid quadruplets, and preservation of pixels/timestamps. The CLI suite rereads
files from Python and verifies associations, rejections, and interruptions. The
real-data protocol checks every exported association and one/four-worker stability,
using `--baseline` to ensure raw detections do not change.

The next step is the optical model and initialization of intrinsics, using
independent synthetic data. A single global homography in the raw image cannot
properly filter fisheye views; outlier selection must account for the projection
model.

Executed validation: **5/5 CTest suites in release and under ASan/UBSan**.
The private `dataset/validation/metric-observations-v1/regression.json` regression
passed: 2,076 images per dataset, 144 target points, 85,776 correspondences for
512 and 118,320 for 1024, with no structural rejection. An independent Python
reader checked every metric position and pixel/point association. Raw detections
match the reference; geometry and correspondences match between one and four
workers. The absence of rejection does not mean reprojection outliers were removed.
