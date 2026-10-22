# Engine choice: targeted reuse or in-house development

Status: recommendations at this stage, not measured performance. Basalt is a
candidate proposed by the user, not a constraint imposed on the project.

## Options

| Approach | Reused components | In-house work and tradeoffs |
| --- | --- | --- |
| Ceres + Basalt components | Camera models and splines, geometric operations | Residuals, RS timing, orchestration, and quality still need development; temporal integration must be verified |
| Ceres + in-house geometry | Solver, Ceres autodiff and manifolds, Eigen | More control and less Basalt coupling; many formulas, domains, and derivatives must be revalidated |
| GTSAM + custom factors | Factor graphs, optimization, geometry, and probabilistic tools | Continuous RS and specific diagnostics still need design; attractive if the objective evolves toward incremental estimation |
| Extend a complete calibrator | Existing initialization, calibration, and infrastructure | Inherited architecture, formats, licenses, and limitations; may require a long-lived fork |
| Develop everything, including the solver | Numerical primitives only | Maximum control at high cost: robustification, manifolds, sparse linear algebra, covariance, parallelism, and validation |

**Recommendation: an in-house application, Ceres for batch optimization, and
selected Basalt components behind adapters.** The project owns the CLI, datasets,
residuals, initialization, timing, metrics, and reports. Basalt must remain
replaceable if an actual difficulty justifies it.

The first GS VI stage uses a small in-house SO(3)+R³ spline behind
`vi_spline.hpp`, with Ceres autodiff for time and variable support. The Basalt
helper with a `double` phase described below would require adaptation, so no extra
Basalt/Sophus dependency is introduced. Temporal derivatives, Jacobians, and
segment transitions are checked against finite differences; independent synthetic
recovery may invalidate this choice. See [the VI model and its limitations](camera-imu.md).

Recency and adoption are signals, not sufficient criteria. Evaluate correctness
of conventions, derivative support, integration cost, dependencies, license,
maintenance, and measurements on our data. There is currently no demonstrated
reason to rewrite a general nonlinear solver.

## Verified points and consequences

[Ceres](https://github.com/ceres-solver/ceres-solver) supports bounded nonlinear
least squares, among other problems. This matches the intended batch estimation.
[GTSAM](https://github.com/borglab/gtsam) structures estimation as factor graphs.
It is a serious alternative, with no speed or accuracy advantage established
here. Do not integrate both in the first version.

Basalt provides a
[Ceres spline helper](https://github.com/VladyslavUsenko/basalt-headers/blob/master/include/basalt/spline/ceres_spline_helper.h).
In the web revision inspected, normalized time `u` is a `double`, whereas knot
parameters are templated. Therefore, do not assume that autodiff automatically
propagates a temporal offset injected into `u`. Options to test are analytically
chained temporal derivatives for offset/readout, or a targeted adaptation of the
evaluation to differentiable scalar time. Segment changes also require support
management.

[Sophus 1](https://github.com/strasdat/Sophus) declares maintenance mode; its README
presents Sophus 2 as a rewrite. Do not automatically replace the version expected
by Basalt with a presumed-compatible successor. Stability of a tested geometry
component may be preferable to an unjustified migration.

OpenCV can initialize and provide a conventional camera–camera comparison. Our
choice is to keep final residuals and VI/RS estimation in the same Ceres problem.
Kalibr, complete Basalt, and the alternatives listed in the
[survey](research.md) serve as experimental references; no fork is selected yet.

## Models and parameters to expose

The first target is `pinhole-radtan` for conventional optics and `pinhole-equi`
for fisheye, with exact formulas and coefficient ordering. Double Sphere and EUCM
are useful extensions if the hardware requires them. The same label in two
libraries does not prove numerical equivalence. Each adapter must be validated by
independent projection and import/export.

The shutter is independent of the optics: a `pinhole-radtan` camera may be global
or rolling shutter without multiplying model names by suffixes. Readout and
offsets have explicit fixed/estimated modes.

IMU scale/non-orthogonality intrinsics remain fixed initially; biases are estimated.
Noise parameters are supplied, or their absence is reported. The central promise
remains reliable extrinsics, not the maximum number of free parameters.

## Experiment required for a final Basalt decision

Before freezing this choice, build a small compiled integration:

1. Project with the selected models and verify Kalibr conventions.
2. Evaluate the spline and its derivatives against an independent analytical trajectory.
3. Verify pose, offset, and readout derivatives, including near knots.
4. Solve a known synthetic VI/RS case, then measure time and allocations.

If this integration requires deep Basalt changes or blocks the derivatives, retain
Ceres and develop only the relevant components. Development time, accuracy, and
maintainability will decide; no performance comparison is claimed before this
experiment.
