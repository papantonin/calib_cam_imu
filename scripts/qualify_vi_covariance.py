#!/usr/bin/env python3
"""Bounded synthetic Monte-Carlo coverage check for the VI covariance.

The fixture and bridge are deliberately reused from tests/vi.py.  This script
does not claim to validate model error: it only checks sampling coverage when
the declared pixel and IMU noise models match the injected noise.
"""
import argparse
import copy
import importlib.util
import json
import math
import random
import subprocess
import sys
from pathlib import Path


PARAMETER_ORDER = [
    "delta_theta_left_x_rad", "delta_theta_left_y_rad", "delta_theta_left_z_rad",
    "delta_translation_x_m", "delta_translation_y_m", "delta_translation_z_m",
    "delta_timeshift_s",
]


def load_fixture_module():
    path = Path(__file__).resolve().parents[1] / "tests" / "vi.py"
    spec = importlib.util.spec_from_file_location("vi_fixture", path)
    module = importlib.util.module_from_spec(spec)
    if spec.loader is None:
        raise RuntimeError(f"cannot load fixture: {path}")
    spec.loader.exec_module(module)
    return module


def rotation_error_left(estimated, truth):
    product = [[sum(estimated[row][k] * truth[column][k] for k in range(3))
                for column in range(3)] for row in range(3)]
    cosine = max(-1.0, min(1.0, (sum(product[i][i] for i in range(3)) - 1.0) / 2.0))
    angle = math.acos(cosine)
    if angle < 1e-10:
        return [
            (product[2][1] - product[1][2]) / 2.0,
            (product[0][2] - product[2][0]) / 2.0,
            (product[1][0] - product[0][1]) / 2.0,
        ]
    scale = angle / (2.0 * math.sin(angle))
    return [scale * (product[2][1] - product[1][2]),
            scale * (product[0][2] - product[2][0]),
            scale * (product[1][0] - product[0][1])]


def cholesky(matrix):
    result = [[0.0] * len(matrix) for _ in matrix]
    for row in range(len(matrix)):
        for column in range(row + 1):
            value = matrix[row][column] - sum(result[row][k] * result[column][k]
                                               for k in range(column))
            if row == column:
                if value <= 0.0:
                    raise ValueError("covariance is not positive definite")
                result[row][column] = math.sqrt(value)
            else:
                result[row][column] = value / result[column][column]
    return result


def mahalanobis_squared(error, covariance):
    lower = cholesky(covariance)
    whitened = []
    for row in range(len(error)):
        value = error[row] - sum(lower[row][column] * whitened[column]
                                 for column in range(row))
        whitened.append(value / lower[row][row])
    return sum(value * value for value in whitened)


def inject_noise(data, generator, pixel_sigma):
    noisy = copy.deepcopy(data)
    for frame in noisy["frames"]:
        for point in frame["points"]:
            point["pixel"] = [value + generator.gauss(0.0, pixel_sigma)
                               for value in point["pixel"]]
    timestamps = [sample["timestamp_ns"] for sample in data["imu"]]
    periods = [(right - left) * 1e-9 for left, right in zip(timestamps, timestamps[1:])
               if right > left]
    if not periods:
        raise ValueError("cannot derive IMU period from timestamps")
    sample_period = sorted(periods)[len(periods) // 2]
    gyro_sigma = noisy["noise"]["gyroscope_noise_density"] / math.sqrt(sample_period)
    accel_sigma = noisy["noise"]["accelerometer_noise_density"] / math.sqrt(sample_period)
    for sample in noisy["imu"]:
        sample["values"] = [value + generator.gauss(0.0, gyro_sigma if axis < 3 else accel_sigma)
                             for axis, value in enumerate(sample["values"])]
    return noisy


def run_trial(bridge, fixture, seed, timeout, pixel_sigma, bootstrap_replicates,
              bootstrap_block_seconds, bootstrap_seed):
    data, truth = fixture.fixture(.013)
    data["pixel_sigma"] = pixel_sigma
    if bootstrap_replicates:
        data.update(bootstrap_replicates=bootstrap_replicates,
                    bootstrap_block_s=bootstrap_block_seconds,
                    bootstrap_seed=bootstrap_seed + seed)
    data = inject_noise(data, random.Random(seed), pixel_sigma)
    command = [str(bridge)]
    try:
        process = subprocess.run(command, input=json.dumps(data), text=True,
                                 capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired as error:
        return {"seed": seed, "status": "timeout", "error_message": str(error)}
    if process.returncode != 0:
        return {"seed": seed, "status": "failed", "error_message":
                f"bridge exit {process.returncode}: {process.stderr.strip()}"}
    result = json.loads(process.stdout)
    recovered = result["bridge_truth_access"]
    transform = recovered["T_cam_imu"]
    truth_transform = truth["T_cam_imu"]
    error = rotation_error_left(transform, truth_transform)
    error += [transform[row][3] - truth_transform[row][3] for row in range(3)]
    error.append(recovered["offset"] - truth["offset"])
    covariance = result["covariance"]
    if covariance.get("status") != "available" or covariance.get("parameter_order") != PARAMETER_ORDER:
        return {"seed": seed, "status": "failed", "error_message": "unusable covariance"}
    sandwich = covariance.get("sandwich", {})
    if sandwich.get("status") != "candidate":
        return {"seed": seed, "status": "failed", "error_message": "unusable sandwich candidate"}
    estimators = {}
    for name, value in (("inverse_hessian", covariance), ("sandwich_candidate", sandwich)):
        matrix = value["matrix"]
        estimators[name] = {
            "standard_deviation": value["standard_deviation"],
            "covariance": matrix,
            "mahalanobis_squared": mahalanobis_squared(error, matrix),
        }
    if bootstrap_replicates:
        bootstrap = result.get("bootstrap", {})
        if bootstrap.get("status") != "candidate":
            return {"seed": seed, "status": "failed", "error_message":
                    f"unusable bootstrap candidate: {bootstrap.get('reason', 'missing')}"}
        matrix = bootstrap["matrix"]
        estimators["temporal_block_bootstrap_candidate"] = {
            "standard_deviation": bootstrap["standard_deviation"],
            "covariance": matrix,
            "mahalanobis_squared": mahalanobis_squared(error, matrix),
            "successful_replicates": bootstrap["successful_replicates"],
        }
    return {"seed": seed, "status": "ok", "error": error, "estimators": estimators}


def wilson_interval(hits, count):
    if count == 0:
        return [None, None]
    center = (hits + 1.9208) / (count + 3.8416)
    radius = 1.96 * math.sqrt(hits * (count - hits) / count + 0.9604) / (count + 3.8416)
    return [max(0.0, center - radius), min(1.0, center + radius)]


def mean_and_std(values):
    if not values:
        return [None, None]
    mean = sum(values) / len(values)
    return [mean, math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))]


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summarize(trials, estimator):
    successful = [trial for trial in trials if trial.get("status") == "ok"]
    marginal_hits = [0] * len(PARAMETER_ORDER)
    joint_hits = 0
    normalized = []
    mahalanobis_hits = 0
    mahalanobis_values = []
    for trial in successful:
        estimate = trial["estimators"][estimator]
        matrix = estimate["covariance"]
        diagonal = [matrix[index][index] for index in range(len(PARAMETER_ORDER))]
        z = [trial["error"][index] / math.sqrt(diagonal[index]) for index in range(len(diagonal))]
        normalized.append(z)
        mahalanobis = estimate["mahalanobis_squared"]
        mahalanobis_values.append(mahalanobis)
        if mahalanobis <= 14.067:
            mahalanobis_hits += 1
        if all(abs(value) <= 1.96 for value in z):
            joint_hits += 1
        for index, value in enumerate(z):
            if abs(value) <= 1.96:
                marginal_hits[index] += 1
    count = len(successful)
    component_coverage = {name: marginal_hits[index] / count if count else None
                          for index, name in enumerate(PARAMETER_ORDER)}
    return {
        "requested_trials": len(trials), "successful_trials": count,
        "failed_trials": len(trials) - count,
        "component_95_percent_coverage": component_coverage,
        "component_coverage_wilson_95_percent": {
            name: wilson_interval(marginal_hits[index], count)
            for index, name in enumerate(PARAMETER_ORDER)},
        "joint_component_box_fraction_at_1_96": joint_hits / count if count else None,
        "joint_chi_square_fraction_at_95_percent_threshold": mahalanobis_hits / count if count else None,
        "normalized_error_mean_and_std": [mean_and_std([row[index] for row in normalized])
                                           for index in range(len(PARAMETER_ORDER))],
        "mahalanobis_squared_median": percentile(mahalanobis_values, 0.5),
        "mahalanobis_squared_p95": percentile(mahalanobis_values, 0.95),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bridge", type=Path, default=Path("build/release/vi_test_bridge"))
    parser.add_argument("--trials", type=int, default=8)
    parser.add_argument("--seed", type=int, default=20261020)
    parser.add_argument("--pixel-sigma", type=float, default=0.8)
    parser.add_argument("--timeout", type=float, default=45.0, help="seconds per trial")
    parser.add_argument("--bootstrap-replicates", type=int, default=0)
    parser.add_argument("--bootstrap-block-seconds", type=float)
    parser.add_argument("--bootstrap-seed", type=int, default=2195)
    parser.add_argument("--require-successes", type=int, default=0,
                        help="return nonzero unless at least this many trials succeed")
    parser.add_argument("--output", type=Path, help="JSON report path; stdout by default")
    args = parser.parse_args()
    if args.trials < 1 or args.trials > 128:
        parser.error("--trials must be between 1 and 128")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.pixel_sigma <= 0:
        parser.error("--pixel-sigma must be positive")
    if args.bootstrap_replicates != 0 and not 8 <= args.bootstrap_replicates <= 128:
        parser.error("--bootstrap-replicates must be 0 or between 8 and 128")
    if bool(args.bootstrap_replicates) != (args.bootstrap_block_seconds is not None):
        parser.error("--bootstrap-replicates and --bootstrap-block-seconds must be provided together")
    if args.bootstrap_block_seconds is not None and args.bootstrap_block_seconds <= 0:
        parser.error("--bootstrap-block-seconds must be positive")
    if args.require_successes < 0 or args.require_successes > args.trials:
        parser.error("--require-successes must be between 0 and --trials")
    if not args.bridge.is_file():
        parser.error(f"bridge not found: {args.bridge}")
    fixture = load_fixture_module()
    trials = []
    for index in range(args.trials):
        seed = args.seed + index
        try:
            trial = run_trial(args.bridge, fixture, seed, args.timeout, args.pixel_sigma,
                              args.bootstrap_replicates, args.bootstrap_block_seconds,
                              args.bootstrap_seed)
        except Exception as error:
            trial = {"seed": seed, "status": "failed", "error_message": str(error)}
        trials.append(trial)
    successful_trials = sum(trial.get("status") == "ok" for trial in trials)
    status = ("campaign_size_100plus" if len(trials) >= 100 else
              "campaign_size_30plus" if len(trials) >= 30 else
              "campaign_size_under_30")
    estimator_names = ["inverse_hessian", "sandwich_candidate"]
    if args.bootstrap_replicates:
        estimator_names.append("temporal_block_bootstrap_candidate")
    report = {
        "schema": "vi_covariance_monte_carlo_v2",
        "seed": args.seed,
        "status": status,
        "noise_model": {"pixel_sigma_px": args.pixel_sigma,
                         "imu_density_to_sample_sigma": "density/sqrt(median timestamp period)"},
        "bootstrap": {"enabled": bool(args.bootstrap_replicates),
                       "replicates": args.bootstrap_replicates,
                       "block_seconds": args.bootstrap_block_seconds,
                       "seed_offset": args.bootstrap_seed,
                       "qualification": "not_qualified_for_default_use"},
        "parameter_order": PARAMETER_ORDER,
        "convention": "R_error = R_est R_truth^T; t_error = t_est - t_truth; offset_error = offset_est - offset_truth",
        "thresholds": {"component_absolute_z": 1.96,
                       "joint_chi_square_7d_95_percent": 14.067},
        "primary_estimator": "inverse_hessian",
        "summary": summarize(trials, "inverse_hessian"),
        "summaries": {name: summarize(trials, name) for name in estimator_names},
        "trials": trials,
    }
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    else:
        sys.stdout.write(text)
    return 0 if successful_trials >= args.require_successes else 2


if __name__ == "__main__":
    raise SystemExit(main())
