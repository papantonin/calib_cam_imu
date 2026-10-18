"""Independent projection/Jacobian oracle. Optional test dependency: numpy, cv2.

Uses OpenCV's compiled implementation, with known identity pose: translation
derivatives then equal derivatives with respect to the camera-frame point.
"""
import json
import subprocess
import sys

import cv2
import numpy as np


def main():
    rng = np.random.default_rng(20261017)
    cases = []
    for model in ("radtan", "equidistant"):
        for n in range(320):
            parameters = np.r_[rng.uniform(200, 1500, 2), rng.uniform(200, 2000, 2),
                               rng.uniform(-1, 1, 4) * [0.03, 0.003, 0.0003, 0.00003]]
            point = np.r_[rng.uniform(-6, 6, 2), 1.] * rng.uniform(.2, 10)
            if n < 16:
                radius = [0., 1e-12, 1e-8, 1e-4 * (1 - 1e-8),
                          1e-4 * (1 + 1e-8), .2, 1., 5.][n % 8]
                point = np.array([radius, 0. if n < 8 else -radius, 1.])
            cases.append(dict(model=model, parameters=parameters.tolist(), point=point.tolist()))
    run = subprocess.run([sys.argv[1]], input=json.dumps(cases), text=True,
                         capture_output=True, check=True)
    results = json.loads(run.stdout)
    if len(results) != len(cases):
        raise AssertionError("incomplete projection results")
    max_pixel_error = 0.
    for case, result in zip(cases, results, strict=True):
        p = np.array(case["parameters"])
        K = np.array([[p[0], 0, p[2]], [0, p[1], p[3]], [0, 0, 1.]])
        point = np.array(case["point"]).reshape(1, 1, 3)
        if case["model"] == "radtan":
            pixel, jac = cv2.projectPoints(point, np.zeros(3), np.zeros(3), K, p[4:])
            point_jac, parameter_jac = jac[:, 3:6], jac[:, 6:14]
        else:
            pixel, jac = cv2.fisheye.projectPoints(point, np.zeros(3), np.zeros(3), K, p[4:])
            point_jac, parameter_jac = jac[:, 11:14], jac[:, :8]
        expected = pixel.reshape(2)
        max_pixel_error = max(max_pixel_error, float(np.max(np.abs(expected-result["pixel"]))))
        np.testing.assert_allclose(result["pixel"], expected, atol=2e-9, rtol=2e-13)
        np.testing.assert_allclose(result["point_jacobian"], point_jac, atol=2e-8, rtol=2e-10)
        np.testing.assert_allclose(result["parameter_jacobian"], parameter_jac, atol=2e-8, rtol=2e-10)
    print(f"OpenCV {cv2.__version__}: {len(cases)} projections and analytic Jacobians agree; "
          f"max pixel difference {max_pixel_error:.3g} px")


if __name__ == "__main__":
    main()
