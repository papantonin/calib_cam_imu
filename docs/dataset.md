# Native dataset and reader contract

Status: the native PNG/CSV reader is implemented, with structural checks and
detection. The temporal-estimation rules below remain a specification for the
future solver. A separate ROS1 converter prepares native inputs; see
[recorded-datasets.md](recorded-datasets.md). MCAP remains future work.

## Directory layout

```text
session/
  cam0/
    1700000000000000000.png
    1700000000033333333.png
  cam1/
    1700000000000000000.png
    1700000000033333333.png
  imu0/
    imu0.csv
```

Camera directories are named `cam0`, `cam1`, and so on. Each PNG is directly in
its directory; its basename is the timestamp in nanoseconds. No camera CSV,
`images/` subdirectory, manifest, or `rig.yaml` is required. A camera-only
acquisition may omit `imu0/`.

Top-level `cam<integer>` directories are discovered in numeric order: cam2 comes
before cam10. Non-contiguous indices are allowed. `--cameras` explicitly selects
and orders streams; without it, all discovered cameras are used. Camera-only mode
announces that it ignores the IMU; camera–IMU mode requires `imu0/imu0.csv`.
A second IMU is not supported in the first version. Source identifiers are
preserved even if a Kalibr export renumbers the chain.

## Images

- `.png` extension; timestamps are nonnegative decimal integers representable as
  int64. Leading zeros are accepted, but duplicates after conversion are detected.
  Fractions, scientific notation, overflows, and nonconforming PNG names are rejected.
- Sort by integer value, never traversal order or modification date. Gaps are
  allowed but measured. Equal timestamps across cameras are normal; two frames at
  the same timestamp in one stream are an error.
- Preserve dimensions, depth, and encoding. PNG mono8/mono16 and RGB8 are supported;
  detector conversion is explicit, stable, and recorded. Mono16 detector conversion
  is `floor(value / 256)`, without gamma or per-frame normalization. Source files
  retain their 16 bits. Other encodings produce an explicit error until supported.
- Read dimensions before large allocations; check multiplication/stride, decode
  limits, and resolution consistency within a stream. A truncated or unreadable
  image is reported with its path, never silently skipped.
- 3840×2160 is a mandatory validation case; do not encode an arbitrary algorithmic
  limit at that resolution.

## Confirmed IMU CSV

```csv
timestamp_ns,wx,wy,wz,ax,ay,az
1700000000000000000,0.012,-0.003,0.001,0.02,-0.01,9.80
1700000000005000000,0.013,-0.002,0.001,0.03,-0.01,9.81
```

A header with unique names is mandatory; columns are identified by name, not
position. The seven names above are required. The initial version rejects unknown
columns rather than guessing their meaning. The separator is a comma, the decimal
separator a period; UTF-8, LF or CRLF, and empty lines are accepted. Units are not
auto-detected. A different existing CSV requires an explicit mapping to be defined.

`w*` is angular velocity in rad/s; `a*` is measured specific force in m/s², with
gravity present at rest, not already compensated acceleration. Axes share the same
right-handed IMU frame. If the manufacturer uses another convention, define a
declared conversion before estimation; do not guess an axis swap.

Timestamps are int64 integers and strictly increasing in the file. A duplicate or
backward jump is an error with a line number; do not conceal it by sorting. NaN/Inf,
missing fields, and malformed lines produce contextual errors. Large gaps or
saturation feed diagnostics; no silent interpolation may fabricate IMU measurements.

## Time and association

The user confirms a common clock whose synchronization must be refined. The
initial model estimates constant offsets, without free clock drift. Timestamps may
be absolute or relative: they need not represent Unix time. Their clock domain and
the exposure reference are separate from the storage format.

Never independently reset every stream to zero. Within one clock, subtract a
common origin using integer arithmetic before conversion to seconds. For a later
extension to separate clocks, retain source origins and apply an explicit model:
`t_reference = scale * (t_source - origin_source) + offset`; record origins, sign,
and direction for every transformation.

Initial model: scale fixed to 1 and estimable constant offsets. Clock drift remains
an extension; detected drift must produce a diagnostic rather than being absorbed
into extrinsics. If epochs differ, request an initial offset and plausible bounds
before concluding that overlap is absent. The relative offset in this equation
must be converted, not blindly copied, into Kalibr's absolute `timeshift_cam_imu`.

The reader does not synchronize streams. The synchronized camera–camera calibrator
associates times with an explicit tolerance and one-to-one matches; default
tolerance is zero, with no pairing by frame index. It reports the match ratio and
actual differences. A nonzero tolerance does not cancel motion between exposures.
Asynchronous cameras require a continuous trajectory and offsets; until that mode
is delivered, reject the request.

Required IMU coverage includes the full modeled exposure of selected images,
offset/readout bounds, and spline support. Report usable intervals after these
constraints, without extrapolating at dataset boundaries.

## Extension to other formats

See [the interfaces](interfaces.md). Initially, only `NativeDirectoryReader` is
implemented. It provides frame references and normalized measurements; the solver
knows neither file paths, topic names, nor ROS APIs.

A future rosbag reader will map a topic to a logical identifier and produce the
same objects with provenance and timestamps. PNG conversion will not be mandatory.
It will explicitly choose sensor or reception timestamps and preserve both when
available. No fake reader, inactive rosbag option, dynamic plugin, or ROS dependency
is added now.
