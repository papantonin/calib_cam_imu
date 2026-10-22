# Validation — checks/detection stage, October 16, 2026

> Historical assessment of the first stage. Mono16 support, the historical border,
> and rosbag validation are detailed in
> [recorded-datasets.md](recorded-datasets.md). The mono16 and border-2 rejections
> described below have since been replaced by support tests; the initial results
> are retained. Later additions are documented in [observations.md](observations.md)
> and [camera-models.md](camera-models.md), with their own test results. Initial
> intrinsic/pose estimation is validated separately in
> [initialization.md](initialization.md); this historical assessment does not cover
> it. GS camera–camera calibration, its tests, and real results are described in
> [camera-camera.md](camera-camera.md). The first GS camera–IMU solver and its
> validations are described in [camera-imu.md](camera-imu.md). Statements below
> that the solver is absent refer to the historical October 16 stage.

## Scope actually executed

Ubuntu 26.04.1 x86_64, AMD Ryzen 5 5600, GCC 15.2, CMake 4.2.3, Ninja 1.13.2.
Dependencies are detailed in [INSTALLATION.md](../INSTALLATION.md). At this
historical stage the executables did not calibrate: no extrinsic, uncertainty, or
calibration-accuracy measurement was available.

| Check | Command | Result |
| --- | --- | --- |
| Optimized C++26 build | `cmake --preset release`, then `cmake --build --preset release` | Passed |
| Validation suites | `ctest --preset release --verbose` | 3/3 passed: core, cli_integration, dependency_lock |
| Memory/UB | `cmake --preset sanitize`, build, then `ctest --preset sanitize` | Core and CLI passed with ASan, leak detection, and UBSan; stop on diagnostic enabled |
| Explicit C++23 fallback | Configure `build/cxx23` with `-DCALIB_CXX_STANDARD=23`, build, CTest | Core and CLI passed with the same GCC 15.2 |
| Installation script | `bash -n scripts/install_dependencies.sh`, then `--sources-only` | Valid syntax; cached archive verified by SHA-256 |
| Locked source | `tests/dependencies.py`, integrated into CTest | 4 tests: valid cache, repair, wrong digest, interrupted download |

The CLI suite runs **56 controlled invocations/processes**, including the image
generator, CLI help, and expected error cases. This is not a count of independent
datasets or validated calibrations.

Tests cover:

- Numeric camera/image ordering, camera selection and duplicates, int64 timestamps,
  duplicate images, and int64 overflow.
- Preservation of 1 ns around timestamps of order 1.7 × 10¹⁸ ns by subtracting
  integers before conversion to seconds.
- Standard and reordered IMU CSV columns, statistics, NaN values, duplicates,
  reversed time order, missing file, and lack of overlap.
- PNG mono8/RGB8, 3840×2160, corrupt payload despite a valid header, resolution
  changes, explicit 16-bit rejection, and insufficient image budget.
- `line_delay`/`lineDelay` import, positive/negative sign, zero, alias conflict,
  duplicate keys, and readout duration. Arithmetic tests for offset sign and corner
  time; **no RS or VI solver was tested**, because none existed at that stage.
- tag36h11/tagStandard41h12 detection, 0/90/180/270° rotations, ordering of four
  corners against independently computed geometry, per-axis error below 1 px on
  these fixtures; decimation 1/2 with difference below 1 px.
- Tag presence after 3×3 mean blur and bounded ±8-intensity noise. This detection
  test does not characterize general bias under motion blur.
- Byte-for-byte identical JSONL outputs with one and four workers; empty images
  retained, out-of-range tags retained as rejections, and a camera without tags reported.
- Refusal to overwrite an existing directory, no final file after a decode error,
  SIGINT after output creation returning code 130 and retaining incomplete state.
- Explicit rejection of an unimplemented calibration and a historical two-bit-border target.

Native tests are in `tests/unit.cpp`. CLI tests use Python without additional
packages and temporary PNGs; tag bitmaps come from official locked
`apriltag_to_image` code. Expected corner positions are computed separately from
known placement and dimensions. The codebook is therefore shared with the detector,
which limits validation independence: real target images remain essential. No user
or external dataset is included. The upstream generator's BSD-2-Clause license is
retained in the dependency archive.

## Detection measurements

Reproduce from the root after a release build:

```sh
python3 scripts/benchmark_detection.py --frames 32 --output build/benchmark-32.json
python3 scripts/benchmark_detection.py --frames 128 --output build/benchmark-128.json
```

One camera, one centered native tag, four repeated rotations, hot-file cache via
hard links. The work budget was set to 4096 MiB to permit four workers at 4K.
Each configuration was measured once without dispersion estimation; there is no
comparison with Kalibr or an older engine. Timing covers indexing, loading,
detection, and observation writing through the summary, not process startup or
the final JSON export.

128 images, decimation 1:

| Resolution | 1-worker time | 4-worker time | Observed speedup | Peak RSS, 4 workers |
| --- | ---: | ---: | ---: | ---: |
| 1280 × 720 | 0.806 s | 0.233 s | 3.46× | 46.1 MiB |
| 1920 × 1080 | 1.719 s | 0.552 s | 3.11× | 91.7 MiB |
| 3840 × 2160 | 10.868 s | 3.230 s | 3.36× | 334.7 MiB |

At 4K with four workers: 32 images take 0.797 s and 334.6 MiB; 128 images take
3.230 s and 334.7 MiB. This experiment confirms that not all images are loaded
simultaneously on this sequence. The file index still grows with their count.
AprilTag internal buffers depend on content; the CLI budget is a scheduling
estimate and **not a strict RSS limit**.

At 4K, decimation 2 and four workers: 128 images in 1.091 s, RSS 123.1 MiB. On
these ideal pixel-aligned tags, maximum per-axis error against known geometry is
below 0.001 px for both decimations; this does not transfer to real images. The
default remains decimation 1. Outputs match across worker counts at equal decimation.

The [raw 32-image measurements](benchmarks/detection-32-2026-10-16.json) and
[128-image measurements](benchmarks/detection-128-2026-10-16.json) preserve
settings, CPU, binary fingerprint, and observation fingerprints. A complete target,
background detail, blur, perspectives, real data, and repeated runs are still
needed before fixing a budget or guaranteed throughput.

## Gaps and next validations

### Deployable installation, October 22, 2026

`bash -n scripts/install.sh` and its help pass. A real installation into a temporary
prefix used
`bash scripts/install.sh --skip-dependencies --skip-tests --jobs 4 --prefix PREFIX`
after prior dependency qualification. It configures and builds the C++23 fallback,
installs and runs `--version` on `calib_cameras` and `calib_imu_camera`, and compiles
the Python report script in memory. The resulting prefix contains exactly five
files: two executables, the report renderer, and two target YAML files. `du` reports
18 MiB; the two commands plus renderer total 18,218,269 bytes. Transitive AprilTag
development install rules are excluded through the `Runtime` component. `ldd`
resolves only expected system libraries and there is no local RPATH/RUNPATH.
After these changes, all 21 release suites pass in 59.72 s. This local check does
not replace testing on a clean machine.

- Matplotlib/Kalibr-style VI report, October 21, 2026: all 20 release suites pass
  in 53.19 s. Synthetic scenarios verify twelve PDF pages, the Matplotlib creator,
  titles, `projection - observation` sign, IMU series, exhaustive evaluated-corner
  count, and absence of leftover `.incomplete` files. The real 1024 replay takes
  40.03 s, 102.09 s CPU, and 325,392 KiB RSS; it plots 977 poses, 10,298 IMU
  samples, and 59,376 x/y residuals. `pdfinfo`, `pdftotext`, and rasterization of
  the eleven graphical pages pass without diagnostics. The positive VI scenario,
  which serializes, launches Python rendering, then rereads the PDF, also passes
  under ASan/UBSan in 186.21 s without diagnostics.
- VI covariance stage, October 20, 2026: all 19 release suites pass in 35.87 s.
  The noisy VI scenario separately passes under C++23/O2 ASan/UBSan in 121.10 s
  without diagnostics. The 100-trial Monte Carlo campaign converges 100/100 in
  249.63 s: inverse Hessian retains 0.80 offset coverage and 0.90 joint χ² coverage;
  block HC0 sandwich retains 0.80 and lowers joint χ² to 0.83. The candidate is
  therefore not qualified. On Oxford, it is explicitly unavailable when nuisance
  elimination needs the regularized limit; the central estimate is unchanged and
  nominal covariance remains available.
- VI bootstrap prototype: eight temporal block-weighted replications of 1 s
  converge 8/8 in 19.83 s on the noisy fixture. The empirical matrix is defined,
  draws are distinct, multiplicities preserved, and the central solution restored
  bit-for-bit. One coverage run checks the three-estimator schema; no bootstrap
  coverage is claimed yet.

- The only two initial compiler warnings came from temporal conversions in AprilTag
  headers. The upstream directory is now marked `SYSTEM` by CMake; warnings remain
  enabled for our code.
- One exact floating-point equality in the IMU-frequency test was replaced with an
  absolute 1e-9 Hz tolerance. No algorithm changed to satisfy this test.
- At the historical stage there was no cache, covariance test, spline, or import
  into a complete Kalibr run. Rig-solver Jacobians, synthetic extrinsic recovery,
  and PDF reading have since been tested; see [camera-camera.md](camera-camera.md).
- Camchain preflight still does not interpret `T_cam_imu`, `T_cn_cnm1`, or
  `timeshift_cam_imu`; it is not a complete validation of that file.
- Grid geometry, historical borders, and the first GS rig solver have since been
  added. Cache, uncertainty, observability qualification, and independent accuracy
  validation on the real rig remain.
