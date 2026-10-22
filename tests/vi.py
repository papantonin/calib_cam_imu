"""Independent analytic motion; IMU derivatives use central differences (10 us/100 us).

The generator uses only Python's standard library and no production trajectory,
projection, pose initializer or solver. World coordinates are the metric grid.
T_cam_imu maps IMU coordinates into camera coordinates; sensor clocks obey
t_imu = t_cam + offset. Intrinsics are fixed and true for this solver test.
"""
import copy
import json
import math
import random
from pathlib import Path
import subprocess
import sys
import tempfile


def transpose(matrix):
    return [list(row) for row in zip(*matrix)]


def multiply(left, right):
    return [[sum(first * second for first, second in zip(row, column))
             for column in zip(*right)] for row in left]


def rotate(matrix, vector):
    return [sum(first * second for first, second in zip(row, vector)) for row in matrix]


def rotation(angles):
    roll, pitch, yaw = angles
    cosine, sine = math.cos, math.sin
    about_x = [[1, 0, 0], [0, cosine(roll), -sine(roll)], [0, sine(roll), cosine(roll)]]
    about_y = [[cosine(pitch), 0, sine(pitch)], [0, 1, 0], [-sine(pitch), 0, cosine(pitch)]]
    about_z = [[cosine(yaw), -sine(yaw), 0], [sine(yaw), cosine(yaw), 0], [0, 0, 1]]
    return multiply(multiply(about_z, about_y), about_x)


def camera_pose(time, mode):
    angles = [.22 * math.sin(1.3 * time), .25 * math.sin(.9 * time + .4), .18 * math.sin(1.7 * time + .2)]
    position = [.15 + .14 * math.sin(1.1 * time), .15 + .11 * math.sin(1.7 * time + .3), -1 + .08 * math.sin(.8 * time)]
    if mode == 'static':
        angles, position = [0, 0, 0], [.15, .15, -1]
    if mode == 'single_axis':
        angles, position = [0, 0, angles[2]], [.15, .15, -1]
    return rotation(angles), position


def bias_profile(time, variable):
    gyro = [.01, -.015, .02]
    accel = [.08, -.06, .04]
    if variable:
        gyro = [gyro[0] + .003 * math.sin(.45 * time),
                gyro[1] + .0025 * (math.cos(.35 * time) - 1),
                gyro[2] + .002 * math.sin(.55 * time + .2)]
        accel = [accel[0] + .04 * math.sin(.30 * time),
                 accel[1] + .03 * (math.cos(.40 * time) - 1),
                 accel[2] + .025 * math.sin(.25 * time + .3)]
    return gyro, accel


def fixture(offset=.02, mode='excited', variable_bias=False):
    epoch = 1700000000000000000
    extrinsic_rotation = rotation([.25, -.18, .30])
    extrinsic_translation = [.03, -.04, .06]
    gyro_bias, accel_bias = bias_profile(0, variable_bias)
    camera = dict(id='cam0', model='radtan', intrinsics=[500., 505., 640., 480.],
                  distortion=[-.035, .008, .001, -.001], resolution=[1280, 960])
    frames = []
    for frame_index in range(181):
        time = frame_index / 20
        world_rotation, position = camera_pose(time, mode)
        inverse_rotation = transpose(world_rotation)
        points = []
        for tag in (0, 2, 4, 5, 12, 14, 16, 17, 30, 32, 34, 35):
            for corner, (horizontal, vertical) in enumerate(((0, 0), (1, 0), (1, 1), (0, 1))):
                world_point = [(tag % 6) * .052 + horizontal * .04, (tag // 6) * .052 + vertical * .04, 0]
                camera_point = rotate(inverse_rotation, [value - origin for value, origin in zip(world_point, position)])
                normalized_x, normalized_y = camera_point[0] / camera_point[2], camera_point[1] / camera_point[2]
                radius2 = normalized_x ** 2 + normalized_y ** 2
                scale = 1 - .035 * radius2 + .008 * radius2 ** 2
                distorted_x = normalized_x * scale + .002 * normalized_x * normalized_y - .001 * (radius2 + 2 * normalized_x ** 2)
                distorted_y = normalized_y * scale + .001 * (radius2 + 2 * normalized_y ** 2) - .002 * normalized_x * normalized_y
                pixel = [500 * distorted_x + 640, 505 * distorted_y + 480]
                assert 0 < pixel[0] < 1280 and 0 < pixel[1] < 960
                points.append(dict(point_id=tag * 4 + corner, pixel=pixel))
        frames.append(dict(timestamp_ns=epoch + round(time * 1e9), points=points))

    def imu_pose(imu_time):
        world_rotation, position = camera_pose(imu_time - offset, mode)
        lever = rotate(world_rotation, extrinsic_translation)
        return multiply(world_rotation, extrinsic_rotation), [value + arm for value, arm in zip(position, lever)]

    imu = []
    for imu_index in range(-10, 911):
        time = imu_index / 100
        world_rotation, position = imu_pose(time)
        derivative_step = 1e-5
        before_rotation, _ = imu_pose(time - derivative_step)
        after_rotation, _ = imu_pose(time + derivative_step)
        derivative = [[(after - before) / (2 * derivative_step) for after, before in zip(after_row, before_row)]
                      for after_row, before_row in zip(after_rotation, before_rotation)]
        angular_matrix = multiply(transpose(world_rotation), derivative)
        gyro = [(angular_matrix[2][1] - angular_matrix[1][2]) / 2,
                (angular_matrix[0][2] - angular_matrix[2][0]) / 2,
                (angular_matrix[1][0] - angular_matrix[0][1]) / 2]
        acceleration_step = 1e-4
        _, before_position = imu_pose(time - acceleration_step)
        _, after_position = imu_pose(time + acceleration_step)
        acceleration = [(after - 2 * current + before) / acceleration_step ** 2
                        for after, current, before in zip(after_position, position, before_position)]
        acceleration[2] += 9.80665
        body_acceleration = rotate(transpose(world_rotation), acceleration)
        sample_gyro_bias, sample_accel_bias = bias_profile(time, variable_bias)
        values = [value + bias for value, bias in zip(gyro, sample_gyro_bias)] + [
            value + bias for value, bias in zip(body_acceleration, sample_accel_bias)]
        imu.append(dict(timestamp_ns=epoch + round(time * 1e9), values=values))
    transform = [row + [translation] for row, translation in zip(extrinsic_rotation, extrinsic_translation)] + [[0, 0, 0, 1]]
    data = dict(camera=camera, frames=frames, imu=imu,
                noise=dict(gyroscope_noise_density=.002, accelerometer_noise_density=.02),
                knot_interval_s=.1, max_offset_s=.05, pixel_sigma=.8, max_iterations=80)
    if variable_bias:
        data['noise'].update(gyroscope_random_walk=.003, accelerometer_random_walk=.03)
        data['bias_knot_interval_s'] = .5
    truth = dict(T_cam_imu=transform, offset=offset, gyro_bias=gyro_bias, accel_bias=accel_bias)
    return data, truth


def run(data, output=None, code=0):
    command = [sys.argv[1]] + ([str(output)] if output else [])
    process = subprocess.run(command, input=json.dumps(data), text=True, capture_output=True, timeout=240)
    assert process.returncode == code, (process.returncode, process.stderr)
    return json.loads(process.stdout) if code == 0 else process.stderr


def check_positive_definite(matrix):
    assert len(matrix) == 7 and all(len(row) == 7 for row in matrix)
    assert all(math.isfinite(value) for row in matrix for value in row)
    assert max(abs(matrix[row][column] - matrix[column][row])
               for row in range(7) for column in range(7)) < 1e-12
    cholesky = [[0.0] * 7 for _ in range(7)]
    for row in range(7):
        for column in range(row + 1):
            value = matrix[row][column] - sum(
                cholesky[row][index] * cholesky[column][index] for index in range(column))
            assert value > 0 if row == column else cholesky[column][column] > 0
            cholesky[row][column] = math.sqrt(value) if row == column else value / cholesky[column][column]


def check_truth(result, truth, check_biases=True):
    recovered = result['bridge_truth_access']
    rotation_error = max(abs(recovered['T_cam_imu'][row][column] - truth['T_cam_imu'][row][column])
                         for row in range(3) for column in range(3))
    translation_error = max(abs(recovered['T_cam_imu'][row][3] - truth['T_cam_imu'][row][3]) for row in range(3))
    assert rotation_error < .005, (rotation_error, recovered)
    assert translation_error < .01, (translation_error, recovered)
    assert abs(recovered['offset'] - truth['offset']) < .002, recovered
    if check_biases:
        assert max(abs(value - expected) for value, expected in zip(recovered['gyro_bias'], truth['gyro_bias'])) < .004, recovered
        assert max(abs(value - expected) for value, expected in zip(recovered['accel_bias'], truth['accel_bias'])) < .04, recovered
    assert abs(math.sqrt(sum(value ** 2 for value in recovered['gravity_world'])) - 9.80665) < 1e-4, recovered
    covariance = result['covariance']
    assert covariance['status'] == 'available', covariance
    assert covariance['parameter_order'] == [
        'delta_theta_left_x_rad', 'delta_theta_left_y_rad', 'delta_theta_left_z_rad',
        'delta_translation_x_m', 'delta_translation_y_m', 'delta_translation_z_m', 'delta_timeshift_s']
    matrix = covariance['matrix']
    assert len(matrix) == 7 and all(len(row) == 7 for row in matrix)
    assert all(math.isfinite(value) for row in matrix for value in row)
    assert all(matrix[index][index] > 0 for index in range(7))
    assert max(abs(matrix[row][column] - matrix[column][row])
               for row in range(7) for column in range(7)) < 1e-12
    assert result['diagnostics']['observability']['status'] == 'full_rank_local_linearization'
    assert 0 < result['diagnostics']['observability']['correlation_eigenvalue_ratio'] <= 1
    assert covariance['nuisance_elimination']['status'] == 'full_rank'
    sandwich = covariance['sandwich']
    assert sandwich['status'] == 'candidate', sandwich
    assert sandwich['qualification'] == 'not_qualified_for_default_use'
    sandwich_matrix = sandwich['matrix']
    check_positive_definite(sandwich_matrix)
    assert 'unqualified candidate' in covariance['statistical_interpretation']
    assert result['diagnostics']['fixed_unreferenced_padding_controls'] == 1


def signed_offset_case(temporary, offset):
    data, truth = fixture(offset)
    folder = Path(temporary) / ('positive' if offset > 0 else 'negative')
    result = run(data, folder)
    check_truth(result, truth)
    rows, inside, yaml_offset = [], False, None
    for line in (folder / 'camchain.yaml').read_text().splitlines():
        if 'T_cam_imu:' in line:
            inside = True
        elif inside and line.strip().startswith('- ['):
            rows.append(json.loads(line.strip()[2:]))
            if len(rows) == 4:
                inside = False
        if 'timeshift_cam_imu:' in line:
            yaml_offset = float(line.split(':', 1)[1])
    assert len(rows) == 4
    assert max(abs(value - expected) for row, truth_row in zip(rows, truth['T_cam_imu'])
               for value, expected in zip(row, truth_row)) < .01
    assert abs(yaml_offset - offset) < .002
    assert (folder / 'report.pdf').read_bytes().startswith(b'%PDF')
    info = subprocess.run(['pdfinfo', str(folder / 'report.pdf')], capture_output=True, text=True, check=True)
    pages = int(next(line.split(':', 1)[1] for line in info.stdout.splitlines() if line.startswith('Pages:')))
    assert pages == 12, info.stdout
    assert 'Creator:         Matplotlib' in info.stdout, info.stdout
    assert not list(folder.glob('*.incomplete'))
    pdf_text = subprocess.run(['pdftotext', str(folder / 'report.pdf'), '-'], capture_output=True, text=True, check=True)
    assert 't_imu = t_cam + timeshift_cam_imu' in pdf_text.stdout
    assert 'local marginalized covariance: available' in pdf_text.stdout.lower()
    assert 'Rotation std [rad]' in pdf_text.stdout
    assert 'imu0: estimated poses in AprilGrid frame' in pdf_text.stdout
    assert 'imu0: estimated motion over time' in pdf_text.stdout
    assert 'imu0: sample inertial rate' in pdf_text.stdout
    assert 'Comparison of predicted and measured specific force' in pdf_text.stdout
    assert 'imu0: acceleration error' in pdf_text.stdout
    assert 'imu0: estimated accelerometer bias' in pdf_text.stdout
    assert 'Comparison of predicted and measured angular velocities' in pdf_text.stdout
    assert 'imu0: angular velocities error' in pdf_text.stdout
    assert 'imu0: estimated gyro bias' in pdf_text.stdout
    assert 'cam0: signed reprojection errors over time' in pdf_text.stdout
    assert 'horizontal residual e_x' in pdf_text.stdout
    assert 'vertical residual e_y' in pdf_text.stdout
    assert 'cam0: reprojection errors' in pdf_text.stdout
    report = result['diagnostics']['report_series']
    assert report['motion_samples'] == report['reprojection_frames'] > 0
    assert report['imu_samples'] == result['diagnostics']['imu_samples'] > 0
    assert report['reprojection_residuals'] == (
        result['diagnostics']['training']['corners'] + result['diagnostics']['held_out']['corners'])
    assert report['residual_definition'] == 'projected_pixel - observed_pixel'
    assert report['residual_x']['samples'] == report['residual_y']['samples'] == report['reprojection_residuals']
    assert result['diagnostics']['imu_samples'] < len(data['imu'])
    print('VI independent truth, biases, transform direction, signed clock offset, YAML/PDF:', offset, flush=True)


def noisy_case():
    noisy, truth = fixture(.013)
    noisy['frames'] = [frame for index, frame in enumerate(noisy['frames'])
                       if not (55 <= index <= 62 or 125 <= index <= 131)]
    generator = random.Random(2195)
    for frame in noisy['frames']:
        for point in frame['points']:
            point['pixel'] = [value + generator.gauss(0, .08) for value in point['pixel']]
        frame['points'][0]['pixel'][0] += 15
    for sample in noisy['imu']:
        sample['values'] = [value + generator.gauss(0, .002 if axis < 3 else .02)
                            for axis, value in enumerate(sample['values'])]
    result = run(noisy)
    check_truth(result, truth)
    nominal = result['covariance']['matrix']
    sandwich = result['covariance']['sandwich']['matrix']
    assert max(abs(nominal[row][column] - sandwich[row][column])
               for row in range(7) for column in range(7)) > 1e-12
    print('VI noisy independent truth with visual outliers and camera gaps passed', flush=True)


def variable_bias_case(temporary):
    data, truth = fixture(.013, variable_bias=True)
    folder = Path(temporary) / 'variable-bias'
    result = run(data, folder)
    check_truth(result, truth, check_biases=False)
    assert result['schema_version'] == 2
    assert result['diagnostics']['bias_model'] == 'piecewise_linear_random_walk'
    trajectory = result['bias_trajectory']
    assert trajectory['type'] == 'piecewise_linear_random_walk'
    assert trajectory['increment_sigma_convention'] == 'sigma_delta = random_walk * sqrt(knot_interval)'
    assert len(trajectory['controls']) == result['diagnostics']['bias_controls'] > 10
    start, interval = trajectory['start_relative_s'], trajectory['knot_interval_s']
    gyro_errors, accel_errors = [], []
    for index, control in enumerate(trajectory['controls'][:-1]):
        expected_gyro, expected_accel = bias_profile(start + index * interval, True)
        gyro_errors.extend(value - expected for value, expected in zip(control[:3], expected_gyro))
        accel_errors.extend(value - expected for value, expected in zip(control[3:], expected_accel))
    gyro_rms = math.sqrt(sum(value * value for value in gyro_errors) / len(gyro_errors))
    accel_rms = math.sqrt(sum(value * value for value in accel_errors) / len(accel_errors))
    assert gyro_rms < .006, gyro_rms
    assert accel_rms < .06, accel_rms
    assert max(max(control[axis] for control in trajectory['controls']) -
               min(control[axis] for control in trajectory['controls']) for axis in range(3)) > 5e-4
    assert max(max(control[axis] for control in trajectory['controls']) -
               min(control[axis] for control in trajectory['controls']) for axis in range(3, 6)) > 5e-3
    text = subprocess.run(['pdftotext', str(folder / 'report.pdf'), '-'], capture_output=True, text=True, check=True).stdout
    assert 'piecewise_linear_random_walk' in text
    assert 'initial bias +/- 3 sigma_rw sqrt(t)' in text
    print('VI time-varying bias recovery and report trajectories passed:', gyro_rms, accel_rms, flush=True)


def bootstrap_case():
    data, _ = fixture(.013)
    generator = random.Random(20261020)
    for frame in data['frames']:
        for point in frame['points']:
            point['pixel'] = [value + generator.gauss(0, data['pixel_sigma']) for value in point['pixel']]
    period = (data['imu'][1]['timestamp_ns'] - data['imu'][0]['timestamp_ns']) * 1e-9
    gyro_sigma = data['noise']['gyroscope_noise_density'] / math.sqrt(period)
    accel_sigma = data['noise']['accelerometer_noise_density'] / math.sqrt(period)
    for sample in data['imu']:
        sample['values'] = [value + generator.gauss(0, gyro_sigma if axis < 3 else accel_sigma)
                            for axis, value in enumerate(sample['values'])]
    central = run(data)
    bootstrapped = run(dict(data, bootstrap_replicates=8, bootstrap_block_s=1.0,
                            bootstrap_seed=2195, bootstrap_max_iterations=40))
    assert bootstrapped['bridge_truth_access'] == central['bridge_truth_access']
    assert bootstrapped['covariance']['status'] == 'available'
    check_positive_definite(bootstrapped['covariance']['matrix'])
    bootstrap = bootstrapped['bootstrap']
    assert bootstrap['status'] == 'candidate', bootstrap
    assert bootstrap['qualification'] == 'not_qualified_for_default_use'
    assert bootstrap['requested_replicates'] == bootstrap['successful_replicates'] == 8
    assert bootstrap['failed_replicates'] == 0
    check_positive_definite(bootstrap['matrix'])
    deltas = [trial['delta'] for trial in bootstrap['trials']]
    assert len({tuple(round(value, 12) for value in delta) for delta in deltas}) > 1
    assert all(sum(trial['multiplicities']) == bootstrap['blocks'] for trial in bootstrap['trials'])
    print('VI bounded temporal block bootstrap candidate and central restoration passed', flush=True)


def guards_case():
    for mode in ('static', 'single_axis'):
        degenerate, _ = fixture(mode=mode)
        run(degenerate, code=3)
    data, _ = fixture()
    cancelled = dict(data, cancelled=True)
    run(cancelled, code=130)
    invalid = copy.deepcopy(data)
    invalid['imu'][5]['timestamp_ns'] = invalid['imu'][4]['timestamp_ns']
    run(invalid, code=2)
    invalid = copy.deepcopy(data)
    invalid['frames'][5]['timestamp_ns'] = invalid['frames'][4]['timestamp_ns']
    run(invalid, code=2)
    invalid = copy.deepcopy(data)
    for sample in invalid['imu']:
        sample['timestamp_ns'] += 100000000000
    run(invalid, code=3)
    invalid = copy.deepcopy(data)
    invalid['camera']['line_delay'] = 1e-5
    run(invalid, code=8)
    invalid = copy.deepcopy(data)
    invalid['noise']['gyroscope_noise_density'] = 0
    run(invalid, code=2)
    invalid = copy.deepcopy(data)
    invalid['noise']['gyroscope_random_walk'] = .001
    run(invalid, code=2)
    invalid = copy.deepcopy(data)
    invalid['noise'].update(gyroscope_random_walk=.001, accelerometer_random_walk=-.01)
    run(invalid, code=2)
    invalid = dict(data, bias_knot_interval_s=.01)
    run(invalid, code=2)
    invalid = copy.deepcopy(data)
    invalid['pixel_sigma'] = 0
    run(invalid, code=2)
    invalid = dict(data, bootstrap_replicates=8, bootstrap_block_s=.1)
    run(invalid, code=2)
    invalid = dict(data, bootstrap_block_s=1.0)
    run(invalid, code=2)
    print('VI static/one-axis degeneracy, timestamps, overlap, cancellation, GS and noise guards passed', flush=True)


def boundary_case():
    invalid, _ = fixture(offset=.02)
    invalid['max_offset_s'] = .005
    assert 'search boundary' in run(invalid, code=3)
    print('VI final offset boundary rejection passed', flush=True)


def main():
    selected = sys.argv[2] if len(sys.argv) > 2 else 'all'
    if selected not in ('all', 'positive', 'negative', 'noisy', 'bias', 'guards', 'boundary', 'bootstrap'):
        raise ValueError(f'Unknown VI test case: {selected}')
    with tempfile.TemporaryDirectory() as temporary:
        if selected in ('all', 'positive'):
            signed_offset_case(temporary, .02)
        if selected in ('all', 'negative'):
            signed_offset_case(temporary, -.02)
        if selected in ('all', 'noisy'):
            noisy_case()
        if selected in ('all', 'bias'):
            variable_bias_case(temporary)
        if selected == 'bootstrap':
            bootstrap_case()
        if selected in ('all', 'guards'):
            guards_case()
        if selected in ('all', 'boundary'):
            boundary_case()


if __name__ == '__main__':
    main()
