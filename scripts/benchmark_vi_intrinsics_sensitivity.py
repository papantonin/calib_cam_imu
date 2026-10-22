#!/usr/bin/env python3
"""Measure VI sensitivity to fixed optical parameters on the independent fixture.

This is a model-sensitivity experiment, not a calibration accuracy certificate.
The pixels and IMU are generated with the unperturbed fixture; only the optical
parameters supplied to the fixed-intrinsics VI solve are changed.
"""
import argparse
import copy
import importlib.util
import json
import math
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
    """Return the left perturbation vector for R_est * R_truth^T."""
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


def cases():
    result = [{"name": "baseline", "changes": {}}]
    for name in ("fx", "fy"):
        for percent in (-0.01, -0.005, 0.005, 0.01):
            result.append({"name": f"{name}_{percent:+.3%}",
                           "changes": {"intrinsics_scale": {name: 1.0 + percent}}})
    for name in ("cx", "cy"):
        for pixels in (-5.0, -2.0, 2.0, 5.0):
            result.append({"name": f"{name}_{pixels:+.1f}px",
                           "changes": {"principal_point_px": {name: pixels}}})
    for name in ("k1", "k2"):
        for percent in (-0.25, -0.10, 0.10, 0.25):
            result.append({"name": f"{name}_{percent:+.0%}",
                           "changes": {"radial_scale": {name: 1.0 + percent}}})
    for name in ("p1", "p2"):
        for value in (-0.0005, 0.0005):
            result.append({"name": f"{name}_{value:+.4f}",
                           "changes": {"tangential_add": {name: value}}})
    result.extend([
        {"name": "combined_positive",
         "changes": {"intrinsics_scale": {"fx": 1.01, "fy": 1.01},
                      "principal_point_px": {"cx": 5.0, "cy": 5.0},
                      "radial_scale": {"k1": 1.25, "k2": 1.25},
                      "tangential_add": {"p1": 0.0005, "p2": 0.0005}}},
        {"name": "combined_mixed",
         "changes": {"intrinsics_scale": {"fx": 0.99, "fy": 1.01},
                      "principal_point_px": {"cx": -5.0, "cy": 5.0},
                      "radial_scale": {"k1": 0.90, "k2": 1.10},
                      "tangential_add": {"p1": -0.0005, "p2": 0.0005}}},
    ])
    return result


def apply_changes(data, changes):
    camera = data["camera"]
    intrinsics = camera["intrinsics"]
    distortion = camera["distortion"]
    for name, scale in changes.get("intrinsics_scale", {}).items():
        intrinsics[("fx", "fy", "cx", "cy").index(name)] *= scale
    for name, pixels in changes.get("principal_point_px", {}).items():
        intrinsics[("fx", "fy", "cx", "cy").index(name)] += pixels
    for name, scale in changes.get("radial_scale", {}).items():
        distortion[("k1", "k2").index(name)] *= scale
    for name, value in changes.get("tangential_add", {}).items():
        distortion[("p1", "p2").index(name) + 2] += value


def metrics(result, truth):
    recovered = result["bridge_truth_access"]
    transform = recovered["T_cam_imu"]
    truth_transform = truth["T_cam_imu"]
    rotation_vector = rotation_error_left(transform, truth_transform)
    translation_vector = [transform[row][3] - truth_transform[row][3] for row in range(3)]
    offset_error = recovered["offset"] - truth["offset"]
    gyro_error = [actual - expected for actual, expected in
                  zip(recovered["gyro_bias"], truth["gyro_bias"])]
    accel_error = [actual - expected for actual, expected in
                   zip(recovered["accel_bias"], truth["accel_bias"])]
    diagnostics = result.get("diagnostics", {})
    covariance = result.get("covariance", {})
    observability = diagnostics.get("observability", {})
    matrix = covariance.get("matrix")
    if covariance.get("status") != "available" or covariance.get("parameter_order") != PARAMETER_ORDER:
        raise ValueError("unusable covariance")
    if not isinstance(matrix, list) or len(matrix) != len(PARAMETER_ORDER) or any(
            not isinstance(row, list) or len(row) != len(PARAMETER_ORDER) for row in matrix):
        raise ValueError("invalid covariance dimensions")
    return {
        "rotation_error_left_rad": rotation_vector,
        "rotation_error_norm_rad": math.sqrt(sum(value * value for value in rotation_vector)),
        "translation_error_m": translation_vector,
        "translation_error_norm_m": math.sqrt(sum(value * value for value in translation_vector)),
        "offset_error_s": offset_error,
        "gyro_bias_error_rad_s": gyro_error,
        "accel_bias_error_m_s2": accel_error,
        "training": diagnostics.get("training"),
        "held_out": diagnostics.get("held_out"),
        "gyro_median_rad_s": diagnostics.get("gyro_median_rad_s"),
        "gyro_p95_rad_s": diagnostics.get("gyro_p95_rad_s"),
        "accel_median_m_s2": diagnostics.get("accel_median_m_s2"),
        "accel_p95_m_s2": diagnostics.get("accel_p95_m_s2"),
        "initial_cost": diagnostics.get("initial_cost"),
        "final_cost": diagnostics.get("final_cost"),
        "covariance": {
            "status": covariance.get("status"),
            "parameter_order": covariance.get("parameter_order"),
            "standard_deviation": covariance.get("standard_deviation"),
            "diagonal": [matrix[index][index] for index in range(len(PARAMETER_ORDER))],
        },
        "observability": {
            "status": observability.get("status"),
            "correlation_eigenvalue_ratio": observability.get("correlation_eigenvalue_ratio"),
        },
    }


def run_case(bridge, fixture, case, timeout):
    data, truth = fixture.fixture(.013)
    data = copy.deepcopy(data)
    apply_changes(data, case["changes"])
    try:
        process = subprocess.run([str(bridge)], input=json.dumps(data), text=True,
                                 capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired as error:
        return {"name": case["name"], "changes": case["changes"],
                "status": "timeout", "error_message": str(error)}
    if process.returncode != 0:
        return {"name": case["name"], "changes": case["changes"], "status": "failed",
                "exit_code": process.returncode,
                "error_message": process.stderr.strip()}
    try:
        result = json.loads(process.stdout)
        measured = metrics(result, truth)
    except (KeyError, IndexError, TypeError, ValueError, json.JSONDecodeError) as error:
        return {"name": case["name"], "changes": case["changes"], "status": "invalid_result",
                "error_message": str(error)}
    return {"name": case["name"], "changes": case["changes"], "status": "ok",
            "exit_code": process.returncode, "metrics": measured}


def summarize(trials):
    successful = [trial for trial in trials if trial["status"] == "ok"]
    if not successful:
        return {"baseline": None, "largest_errors": None}
    baseline = next((trial for trial in successful if trial["name"] == "baseline"), None)
    keys = ("rotation_error_norm_rad", "translation_error_norm_m", "offset_error_s")
    largest = {}
    for key in keys:
        trial = max(successful, key=lambda item: abs(item["metrics"][key]))
        largest[key] = {"case": trial["name"], "value": trial["metrics"][key]}
    held_out = [trial for trial in successful if trial["metrics"].get("held_out")]
    if held_out:
        trial = max(held_out, key=lambda item: item["metrics"]["held_out"]["median_px"])
        largest["held_out_median_px"] = {
            "case": trial["name"], "value": trial["metrics"]["held_out"]["median_px"]}
    return {"baseline": baseline["metrics"] if baseline else None, "largest_errors": largest}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bridge", type=Path, default=Path("build/release/vi_test_bridge"))
    parser.add_argument("--output", type=Path, help="JSON report path; stdout by default")
    parser.add_argument("--timeout", type=float, default=45.0,
                        help="seconds per case (default: 45)")
    parser.add_argument("--max-cases", type=int,
                        help="maximum cases including baseline (default: all)")
    parser.add_argument("--require-successes", type=int, default=0,
                        help="return nonzero unless at least this many cases succeed")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    available_cases = cases()
    if args.max_cases is None:
        args.max_cases = len(available_cases)
    if args.max_cases < 1 or args.max_cases > len(available_cases):
        parser.error(f"--max-cases must be between 1 and {len(available_cases)}")
    if args.require_successes < 0 or args.require_successes > args.max_cases:
        parser.error("--require-successes must be between 0 and --max-cases")
    if not args.bridge.is_file():
        parser.error(f"bridge not found: {args.bridge}")

    fixture = load_fixture_module()
    selected_cases = available_cases[:args.max_cases]
    trials = []
    for case in selected_cases:
        try:
            trials.append(run_case(args.bridge, fixture, case, args.timeout))
        except Exception as error:
            trials.append({"name": case["name"], "changes": case["changes"],
                           "status": "harness_error", "error_message": str(error)})
    successful = sum(trial["status"] == "ok" for trial in trials)
    report = {
        "schema": "vi_intrinsics_sensitivity_v1",
        "status": "completed" if successful == len(trials) else "completed_with_failures",
        "interpretation": "Sensitivity experiment only; no universal accuracy verdict.",
        "fixture": "tests/vi.py fixture(offset=0.013), unperturbed pixels and IMU",
        "fixed_intrinsics": True,
        "parameter_order": PARAMETER_ORDER,
        "convention": "R_error = R_est R_truth^T; translations and offset are estimate minus truth",
        "requested_cases": len(trials),
        "successful_cases": successful,
        "failed_cases": len(trials) - successful,
        "summary": summarize(trials),
        "trials": trials,
    }
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    else:
        sys.stdout.write(text)
    return 0 if successful >= args.require_successes else 2


if __name__ == "__main__":
    raise SystemExit(main())
