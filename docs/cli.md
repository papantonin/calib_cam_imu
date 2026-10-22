# Proposed complete CLI

> This document includes future options. Synchronized GS camera–camera calibration
> is available with `--models`, `--target`, and `--output`, together with
> `--check-dataset`, `--detect-only`, and `--initialize-only`. The
> [README](../README.md) and [camera-camera.md](camera-camera.md) describe the
> delivered options exactly. Undelivered options are rejected by the CLI; IMU and
> RS features remain under design in this proposed interface.

The validated executables are `calib_cameras` and `calib_imu_camera`. Calibration
mode is explicit, arguments follow Kalibr vocabulary, and a global configuration
file remains optional. Beyond the available subset described above, detailed
options are still being designed.

## Camera–camera

```sh
calib_cameras --dataset ./session --models pinhole-radtan pinhole-radtan --target aprilgrid.yaml
```

Cameras are discovered as `cam0`, `cam1`, and so on, in numeric order. To select
streams or change the reference camera:

```sh
calib_cameras --dataset ./session --cameras cam1 cam0 --models pinhole-equi pinhole-equi --target aprilgrid.yaml
```

Provide one model per camera in the same order; an incorrect cardinality is an
error. The first selected camera fixes the camera–camera reference frame. A single
stream remains allowed for intrinsics but produces no inter-camera extrinsic.
For known intrinsics, use `--cam camchain.yaml` instead of `--models`, with fixed
intrinsics and inter-camera extrinsics estimated by default. Import must map each
camera to its directory unambiguously.

The camera–camera synchronization assumption is stated in the summary. By default,
timestamps must match exactly; `--sync-tolerance-ms` sets a tolerance. The future
`--camera-timing estimate-offsets` mode will require a continuous trajectory and
will be exposed only when implemented.

## Camera–IMU, Kalibr-like workflow

With an existing camera calibration:

```sh
calib_imu_camera --dataset ./session --cam camchain.yaml --imu imu.yaml --target aprilgrid.yaml
```

`--cam` describes camera calibration; `--imu` describes IMU noise and properties.
**Measurement CSV is discovered at `imu0/imu0.csv`**, not passed through `--imu`.
Imported intrinsics and inter-camera extrinsics are fixed by default; `T_cam_imu`,
biases, and camera–IMU offsets are estimated. Temporal correction is enabled by
default because sensors share a clock whose synchronization needs refinement.
`--time-offset-ms cam0=0` fixes a known offset; `--time-offset-ms cam0=estimate`
requests estimation with explicit configurable bounds. A default search window
must be justified and published.

To avoid a preliminary command when starting from scratch:

```sh
calib_imu_camera --dataset ./session --models pinhole-radtan pinhole-radtan --imu imu.yaml --target aprilgrid.yaml
```

Camera initialization and VI calibration are then chained internally. This mode
must be explicitly validated and delivered; it is not a mere syntax variant of
the fixed-camera solver. The data must support both estimates; poor dynamics do
not replace good optical coverage. One command does not guarantee that one
sequence is sufficient.

`--cam` and `--models` are mutually exclusive. In VI mode with `--cam`,
`--camera-parameters refine` releases camera calibration after initialization;
the default is `fixed`. With `--models`, the default and only consistent mode is
`refine`. `--camera-parameters` is reserved for the VI command. In camera-only
mode, the distinct `--intrinsics fixed/refine` option controls imported intrinsics;
inter-camera extrinsics remain estimated. The report lists fixed/free blocks.

## Argument mapping

| Argument | Proposed meaning |
| --- | --- |
| `--dataset PATH` | Native dataset root; initially the only implemented format |
| `--cameras cam0 cam1` | Subset and order; defaults to numeric discovery |
| `--models ...` | One projection/distortion model per camera, parameters to estimate |
| `--cam FILE` | Existing camera calibration, native format or validated Kalibr subset |
| `--imu FILE` | IMU profile including noise; distinct from the measurement CSV |
| `--target FILE` | Target geometry and family; mandatory and reusable |
| `--output DIR` | New result directory; otherwise a dated directory under `session/calibration/` |
| `--threads N` | Global CPU budget, automatic by default |
| `--memory-budget-mb N` | Image and estimation budget, with a published default |
| `--quality standard/thorough` | Routine diagnostics or deeper stability analyses |
| `--config FILE` | Optional preset grouping recurring arguments |

There is no `--topics` for an image directory. No inert `--bag` or format option
exists yet. A future rosbag reader will add stream selection without changing
optical, temporal, or scientific options.

For a Kalibr camchain with `cam0`, `cam1` keys, mapping to same-named directories
is direct. For a different mapping, require an explicit map in the `--config`
preset; do not rely only on order or a legacy `rostopic`. Results always preserve
source identifiers.

## Rolling shutter in the camchain

`camchain.yaml` is sufficient input for reusing a conventional Kalibr RS
calibration: `line_delay` in seconds per row, resolution, and optical parameters.
The reader activates RS for a nonzero value, applies
`t_corner = t_image + offset + y * line_delay`, and fixes the delay by default.
No additional sensor file is requested.

```sh
calib_imu_camera --dataset ./session --cam camchain.yaml --imu imu.yaml --target aprilgrid.yaml
```

`--refine-line-delay cam0` enables optional joint temporal refinement with explicit
bounds in options/preset. `--camera-parameters refine` does not release the RS
delay by itself. An absent value without another declaration means announced GS;
zero is the GS limit. `lineDelay` may be accepted as an import alias, but export
writes the canonical `line_delay` key.

For a first estimate without an existing calibration, retain `--shutter
cam0=rolling`, `--readout-ms cam0=estimate`, `--readout-bounds-ms cam0=0:40`, and
the required convention settings. Illustrative bounds must be adapted to the
hardware. Readout and per-row delay are two representations of one degree of
freedom, not two free parameters. Measured parameters are written to the camchain
for the next stage.

Sensor-specific conventions and any image transforms are stored in an optional
`rolling_shutter` block in the same camera input, specific to our extended format.
The [RS/IMU specification](rolling-shutter.md) describes import, exact reference
conversions, and reuse limitations across sensor modes.

## Less typing without mandatory configuration

Explicit arguments are enough for first use. For a recurring setup, a preset is
a convenience:

```sh
calib_imu_camera --dataset ./session --config setup.yaml
```

The preset contains the same fields as options and references the target, IMU
noise, and camera calibration. Precedence is CLI options > preset > defaults.
Incompatible imported metadata must be reported and the replacement value logged;
never merge it implicitly across two frames/models. Unknown keys and repeated CLI
assignments for the same field are errors. File paths are relative to that file;
command-line paths are relative to the current directory. There is no hidden YAML
search.

Inspection, validation, and report regeneration may be exposed separately. They
are never mandatory calibration stages. A future multi-sequence option must keep
trajectories and time origins separate while sharing geometry; it is not exposed
prematurely.

## Results and execution contract

Each command performs checks, detection, estimation, quality assessment, and PDF
reporting. Outputs are `calibration.yaml`, `camchain.yaml` or compatible
`camchain-imucam.yaml`, `quality.json`, `report.pdf`, `resolved-config.yaml`, and
`run.json`. Kalibr export must not hide any loss of RS parameters.

Create a distinct result directory for every run without overwriting. A compatible
detection cache may be shared; writes are atomic and complete/interrupted state is
explicit. Proposed exit codes remain: 0 accepted, 2 invalid input, 3 insufficient
data/quality rejected, 4 numerical failure, 5 I/O error, 6 warnings, 7 required
export/report failure. A PDF failure does not delete a computed solution, but the
run is not completely successful. Logs go to stderr.
