# Module boundaries — proposal

The following names define responsibilities, not classes that have already been
written or a public ABI to freeze. Only the abstractions required by the native
reader and the first solver will be introduced during implementation.

## Input and observations

| Object / operation | Contract |
| --- | --- |
| `DatasetSource` | Source type and URI; initially, only the native source is accepted |
| `DatasetReader::catalog()` | Available identifiers, stream types, clock domains, and metadata |
| `DatasetReader::frames(camera)` | Ordered `FrameRef` cursor without decoding every image |
| `DatasetReader::decode(frame)` | Owned image buffer with dimensions, stride, and format; parallel operation according to declared capabilities |
| `DatasetReader::imu(imu_id, interval)` | Measurement cursor over a source-time interval, without resampling |
| `FrameRef` | Stable ID, sensor, timestamp_ns, clock, optional exposure, and a location opaque to the solver |
| `ImuSample` | Source timestamp, rad/s, m/s², frame, and line/message provenance |
| `CornerObservation` | Frame, camera, target/tag/corner ID, sensor pixel, quality, and covariance when justified |

The source timestamp and its domain are separate from relative optimization time.
An unknown field remains optional; never fabricate an exposure or hardware
timestamp. Explicit RAII ownership keeps the decoded buffer valid during detection.
No pointer may refer to temporary decoder memory.

The native reader permits several independent decodes. A future container may
offer a sequential cursor and bounded prefetching: the scheduler must respect the
reader capabilities and must not require fast random access to every source.
Errors carry a code, stream, and location before being rendered by the CLI.
The source and configuration fingerprints identify caches.

## Configuration and estimation

| Object | Responsibility |
| --- | --- |
| `CalibrationRequest` | Camera or VI mode, stream selection, target, fixed/free parameters, and resources |
| `CameraSpec` | Projection and distortion, ordered parameters, and valid domain |
| `ShutterSpec` | GS/RS, readout direction, timestamp, exposure, and bounded readout |
| `ClockSpec` | Reference, origins, known/estimated offsets, and bounds |
| `TargetGeometry` | Mapping from ID/corner to metric 3D point; family and printed geometry |
| `ObservationSet` | Normalized corners and IMU data; independent of original storage |
| `CalibrationState` | Typed transforms, intrinsics, temporal parameters, biases, and trajectory |
| `CalibrationResult` | Estimated state, convergence, fixed/free blocks, covariances, and provenance |
| `QualityAssessment` | Measurements, versioned criteria, limitations, verdict, and recommendations |

The CLI layer resolves arguments into a single request before any optimization.
For camera–camera calibration, relative placement is initialized automatically
from observations: no theoretical-placement argument or mechanical prior is
planned. Exported extrinsics specify transform direction, frames, and units so
the user can perform an external comparison. Camchain import also constructs
`ShutterSpec` from `line_delay` and any extensions; the dataset does not provide
an additional sensor file. Mathematical engines do not read YAML. Exporters and
the report consume a result and its diagnostics without accessing the solver or
recomputing the calibration.

## Basalt / Ceres boundary

Persistent types belong to the project: named parameter arrays, transforms with
frames and conventions, and versioned results. Adapters may use Eigen and Sophus
internally. No Basalt or Ceres type is exposed in the dataset or output formats.

Camera models and trajectories provide testable mathematical operations:
projection, valid domain, pose at t, angular velocity, and acceleration. Residual
parameters are typed/templated at compile time to preserve autodiff; do not impose
a virtual call per corner in the critical loop. Model selection happens once when
the problem is built.

A Ceres adapter assembles visual, IMU, bias, and prior residuals and selects the
manifolds. There is initially a single backend. Preserving a replaceable boundary
does not mean building and maintaining two solvers in parallel.

A shared component computes each corner's projection time and is tested separately
from projection. The trajectory knows which knots are required over the permitted
interval: offset and readout must not change the support unless problem assembly
accounts for it.

## Extensions without overengineering

To add a format, implement a reader and its selection in a compiled factory, then
add reader conformance tests. Keep the same observations, residuals, and exports.
A loadable-plugin ABI is not currently justified.

To add an optical model, provide projection, derivatives, domain, parameter order,
initialization, and export rules; a formula alone is insufficient. To replace the
Basalt splines, reproduce the pose/derivative contract, conventions, and tests
before comparing performance and memory.
