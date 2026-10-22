#!/usr/bin/env python3
"""Render the camera-IMU calibration report with headless Matplotlib."""
import json
import math
from pathlib import Path
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
import numpy as np


KALIBR_PAGE = (6.4, 4.8)


def scalar(value):
    if value is None:
        return 'unavailable'
    if isinstance(value, float):
        return f'{value:.8g}'
    return str(value)


def quaternion_matrix(quaternion):
    w, x, y, z = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_euler_zyx(rotation):
    return [
        math.atan2(rotation[2, 1], rotation[2, 2]),
        math.asin(np.clip(-rotation[2, 0], -1, 1)),
        math.atan2(rotation[1, 0], rotation[0, 0]),
    ]


def finish(pdf, figure):
    pdf.savefig(figure, dpi=160)
    plt.close(figure)


def summary_page(pdf, report):
    diagnostics = report['diagnostics']
    covariance = diagnostics.get('covariance') or {}
    covariance_available = covariance.get('status') == 'available'
    camera = report['camera']
    figure = plt.figure(figsize=KALIBR_PAGE)
    figure.suptitle('Calibration results and configuration', x=.05, y=.97, ha='left', fontsize=12,
                    fontweight='bold')
    lines = [
        f"Camera: {report['camera_id']} | Global shutter | AprilGrid/world frame",
        f"Quality: provisional; local marginalized covariance: {'available' if covariance_available else 'unavailable'}",
        'T_cam_imu maps IMU coordinates into camera coordinates.',
        'Time convention: t_imu = t_cam + timeshift_cam_imu',
        f"timeshift_cam_imu [s]: {scalar(report['timeshift_cam_imu'])}",
        'line_delay [s/line]: 0 (global shutter)',
        'first gyro bias control [rad/s]: ' + '  '.join(scalar(value) for value in report['gyro_bias']),
        'first accel bias control [m/s2]: ' + '  '.join(scalar(value) for value in report['accel_bias']),
        'gravity world [m/s2]: ' + '  '.join(scalar(value) for value in report['gravity_world']),
        '',
        'T_cam_imu:',
    ]
    lines.extend('  '.join(f'{value: .8g}' for value in row) for row in report['T_cam_imu'])
    lines.extend([
        '',
        f"Solver status: {scalar(diagnostics.get('status'))}",
        f"Iterations: {scalar(diagnostics.get('iterations'))}",
        f"Initial cost: {scalar(diagnostics.get('initial_cost'))}",
        f"Final cost: {scalar(diagnostics.get('final_cost'))}",
        '',
        f"Bias model: {diagnostics.get('bias_model', 'unknown')}; controls: {diagnostics.get('bias_controls', 'unknown')}",
        '',
        f"Camera model / distortion: {camera['camera_model']} / {camera['distortion_model']}",
        'Intrinsics [fx fy cx cy]: ' + '  '.join(scalar(value) for value in camera['intrinsics']),
        'Distortion coefficients: ' + '  '.join(scalar(value) for value in camera['distortion_coeffs']),
        f"Resolution: {camera['resolution'][0]} x {camera['resolution'][1]}",
        'IMU noise density gyro / accel: '
        f"{scalar(diagnostics.get('gyro_noise_density'))} / {scalar(diagnostics.get('accel_noise_density'))}",
    ])
    if covariance_available:
        deviation = covariance['standard_deviation']
        lines.append('Rotation std [rad]: ' + '  '.join(scalar(value) for value in deviation['rotation_left_rad']))
        lines.append('Translation std [m]: ' + '  '.join(scalar(value) for value in deviation['translation_m']))
        lines.append('Time std [s]: ' + scalar(deviation['timeshift_s']))
    else:
        lines.append('Local covariance unavailable: see calibration.json observability status.')
    if 'training' in diagnostics and 'held_out' in diagnostics:
        lines.extend([
            '',
            'Visual RMS training / held-out [px]: '
            f"{scalar(diagnostics['training'].get('rms_px'))} / {scalar(diagnostics['held_out'].get('rms_px'))}",
            f"Gyroscope RMS [rad/s]: {scalar(diagnostics.get('gyro_rms_rad_s'))}",
            f"Accelerometer RMS [m/s2]: {scalar(diagnostics.get('accel_rms_m_s2'))}",
            'Held-out frames are evaluated without refitting their target poses.',
            'Local covariance assumes fixed optical estimates and declared independent noise.',
        ])
    figure.text(.05, .925, '\n'.join(lines), va='top', family='monospace', fontsize=6.2, linespacing=1.12)
    finish(pdf, figure)


def motion_arrays(report):
    motion = report['motion']
    times = np.asarray([sample['time_s'] for sample in motion], dtype=float)
    camera_positions = np.asarray([sample['position_world_m'] for sample in motion], dtype=float)
    camera_rotations = np.asarray(
        [quaternion_matrix(sample['rotation_world_camera_wxyz']) for sample in motion], dtype=float)
    transform_cam_imu = np.asarray(report['T_cam_imu'], dtype=float)
    rotation_cam_imu = transform_cam_imu[:3, :3]
    translation_cam_imu = transform_cam_imu[:3, 3]
    imu_rotations = camera_rotations @ rotation_cam_imu
    imu_positions = camera_positions + np.einsum('nij,j->ni', camera_rotations, translation_cam_imu)
    return times, imu_positions, imu_rotations


def trajectory_page(pdf, positions, rotations):
    figure = plt.figure(figsize=KALIBR_PAGE)
    figure.suptitle('imu0: estimated poses in AprilGrid frame', fontsize=11)
    axis = figure.add_subplot(111, projection='3d')
    axis.plot(positions[:, 0], positions[:, 1], positions[:, 2], color='black', linewidth=1)
    span = max(*(float(np.ptp(positions[:, index])) for index in range(3)), .01)
    glyph = .06 * span
    stride = max(1, len(positions) // 45)
    colors = ('#d62728', '#2ca02c', '#1f77b4')
    for index in range(0, len(positions), stride):
        start = positions[index]
        for local_axis, color in enumerate(colors):
            end = start + glyph * rotations[index, :, local_axis]
            axis.plot((start[0], end[0]), (start[1], end[1]), (start[2], end[2]), color=color,
                      linewidth=.65)
    center = np.mean([positions.min(axis=0), positions.max(axis=0)], axis=0)
    half_span = .55 * span + glyph
    axis.set_xlim(center[0] - half_span, center[0] + half_span)
    axis.set_ylim(center[1] - half_span, center[1] + half_span)
    axis.set_zlim(center[2] - half_span, center[2] + half_span)
    axis.set_box_aspect((1, 1, 1))
    axis.set_xlabel('world X [m]')
    axis.set_ylabel('world Y [m]')
    axis.set_zlabel('world Z [m]')
    axis.view_init(elev=28, azim=-58)
    axis.grid(True, alpha=.3)
    figure.text(.02, .02, 'T_world_imu = T_world_cam T_cam_imu; axes: X red, Y green, Z blue.', fontsize=6.5)
    figure.subplots_adjust(top=.91, bottom=.08, left=.02, right=.97)
    finish(pdf, figure)


def component_page(pdf, times, positions, rotations):
    angles = np.asarray([rotation_euler_zyx(value) for value in rotations], dtype=float)
    angles = np.rad2deg(np.unwrap(angles, axis=0))
    figure, axes = plt.subplots(3, 2, figsize=KALIBR_PAGE, sharex=True)
    figure.suptitle('imu0: estimated motion over time', fontsize=10)
    for index, name in enumerate(('X', 'Y', 'Z')):
        axes[index, 0].plot(times, positions[:, index], color='#1769aa', linewidth=1)
        axes[index, 0].set_title(f'position {name}')
        axes[index, 0].set_ylabel('m')
        axes[index, 1].plot(times, angles[:, index], color='#b3261e', linewidth=1)
        axes[index, 1].set_title(('roll X', 'pitch Y', 'yaw Z')[index])
        axes[index, 1].set_ylabel('deg')
        for axis in axes[index]:
            axis.grid(True, alpha=.3)
    axes[2, 0].set_xlabel('time [s]')
    axes[2, 1].set_xlabel('time [s]')
    figure.subplots_adjust(top=.90, bottom=.10, left=.09, right=.97, hspace=.48, wspace=.30)
    finish(pdf, figure)


def imu_arrays(report):
    samples = report['imu']
    times = np.asarray([sample['time_s'] for sample in samples], dtype=float)
    times -= times[0]
    fields = {}
    for name in ('measured_gyro_rad_s', 'predicted_gyro_rad_s', 'gyro_residual_rad_s',
                 'measured_accel_m_s2', 'predicted_accel_m_s2', 'accel_residual_m_s2',
                 'gyro_bias_rad_s', 'accel_bias_m_s2'):
        fields[name] = np.asarray([sample[name] for sample in samples], dtype=float)
    return times, fields


def imu_rate_page(pdf, times):
    intervals_ms = np.diff(times) * 1000
    figure, axis = plt.subplots(figsize=KALIBR_PAGE)
    figure.suptitle('imu0: sample inertial rate', fontsize=10)
    axis.scatter(times[1:], intervals_ms, s=2, marker='x', color='#1565c0', linewidths=.4,
                 rasterized=True)
    axis.text(.04, .91, f'avg dt (ms) = {np.mean(intervals_ms):.3f} +- {np.std(intervals_ms):.4f}',
              transform=axis.transAxes, fontsize=8)
    axis.set_xlim(times[0], times[-1])
    axis.set_ylim(0, max(float(np.max(intervals_ms)) * 1.05, 1e-3))
    axis.set_xlabel('time (s)')
    axis.set_ylabel('sample interval (ms)')
    axis.grid(True, alpha=.3)
    figure.subplots_adjust(top=.88, bottom=.14, left=.12, right=.97)
    finish(pdf, figure)


def vector_comparison_page(pdf, title, times, predicted, measured, ylabel):
    figure, axes = plt.subplots(3, 1, figsize=KALIBR_PAGE, sharex=True)
    figure.suptitle(title, fontsize=9)
    for index, axis in enumerate(axes):
        axis.plot(times, predicted[:, index], color='#1565c0', linewidth=1.1, label='model prediction')
        axis.plot(times, measured[:, index], linestyle='', marker='x', markersize=.8, markeredgewidth=.35,
                  color='#ef6c00', alpha=.55, label='imu0', rasterized=True)
        axis.set_ylabel(f'{ylabel} {"xyz"[index]}')
        axis.grid(True, alpha=.3)
    axes[0].legend(loc='upper right', frameon=False, fontsize=6)
    axes[-1].set_xlabel('time (s)')
    figure.subplots_adjust(top=.89, bottom=.11, left=.12, right=.97, hspace=.18)
    finish(pdf, figure)


def vector_error_page(pdf, title, times, residuals, ylabel, sigma):
    figure, axes = plt.subplots(3, 1, figsize=KALIBR_PAGE, sharex=True)
    figure.suptitle(title, fontsize=10)
    bound = 3 * sigma
    for index, axis in enumerate(axes):
        axis.plot(times, residuals[:, index], color='#1565c0', linewidth=.55, rasterized=True)
        axis.axhline(bound, color='#d62728', linestyle='--', linewidth=.7)
        axis.axhline(-bound, color='#d62728', linestyle='--', linewidth=.7)
        axis.axhline(0, color='.25', linewidth=.5)
        axis.set_ylabel(f'{ylabel} {"xyz"[index]}')
        axis.grid(True, alpha=.3)
    axes[-1].set_xlabel('time (s)')
    figure.text(.73, .935, 'red: declared +/- 3 sample sigma', fontsize=6)
    figure.subplots_adjust(top=.89, bottom=.11, left=.12, right=.97, hspace=.18)
    finish(pdf, figure)


def bias_page(pdf, title, times, bias, ylabel, random_walk):
    figure, axes = plt.subplots(3, 1, figsize=KALIBR_PAGE, sharex=True)
    figure.suptitle(title, fontsize=10)
    bounds = 3 * random_walk * np.sqrt(np.maximum(times - times[0], 0))
    for index, axis in enumerate(axes):
        axis.plot(times, bias[:, index], color='#1565c0', linewidth=1.0)
        if random_walk > 0:
            axis.plot(times, bias[0, index] + bounds, color='red', linestyle='--', linewidth=.7)
            axis.plot(times, bias[0, index] - bounds, color='red', linestyle='--', linewidth=.7)
        axis.set_ylabel(f'{ylabel} {"xyz"[index]}')
        axis.grid(True, alpha=.3)
    axes[-1].set_xlabel('time (s)')
    figure.text(.55, .03, ('red: initial bias +/- 3 sigma_rw sqrt(t)' if random_walk > 0
                           else 'constant-bias model'), fontsize=6)
    figure.subplots_adjust(top=.89, bottom=.11, left=.12, right=.97, hspace=.18)
    finish(pdf, figure)


def residual_arrays(report):
    chunks = []
    times = []
    held_out = []
    for frame in report['reprojection']:
        residuals = np.asarray(frame['residuals_px'], dtype=float)
        if residuals.size == 0:
            continue
        chunks.append(residuals.reshape((-1, 2)))
        times.append(np.full(len(residuals), frame['time_s'], dtype=float))
        held_out.append(np.full(len(residuals), frame['held_out'], dtype=bool))
    if not chunks:
        raise ValueError('VI report has no reprojection residuals')
    return np.concatenate(chunks), np.concatenate(times), np.concatenate(held_out)


def residual_time_page(pdf, report, residuals, times, held_out):
    figure, axes = plt.subplots(2, 1, figsize=KALIBR_PAGE, sharex=True)
    figure.suptitle('cam0: signed reprojection errors over time', fontsize=10)
    figure.text(.5, .925, f"projected - observed; all {len(residuals)} corners in "
                f"{len(report['reprojection'])} frames", ha='center', fontsize=6.5)
    training = ~held_out
    limit = max(float(np.max(np.abs(residuals))), .05) * 1.05
    for component, axis in enumerate(axes):
        axis.axhline(0, color='.25', linewidth=.7)
        axis.scatter(times[training], residuals[training, component], s=2, color='#1565c0', alpha=.35,
                     linewidths=0, rasterized=True)
        axis.scatter(times[held_out], residuals[held_out, component], s=2, color='#ef6c00', alpha=.45,
                     linewidths=0, rasterized=True)
        axis.set_ylim(-limit, limit)
        axis.set_ylabel('pixels')
        axis.set_title(('horizontal residual e_x', 'vertical residual e_y')[component])
        axis.grid(True, alpha=.25)
    axes[1].set_xlabel('time [s]')
    axes[0].legend(handles=[Line2D([], [], marker='o', linestyle='', color='#1565c0', label='training'),
                            Line2D([], [], marker='o', linestyle='', color='#ef6c00', label='held-out')],
                   loc='upper right', frameon=False)
    figure.subplots_adjust(top=.87, bottom=.12, left=.12, right=.97, hspace=.34)
    finish(pdf, figure)


def residual_distribution_page(pdf, report, residuals, times):
    figure, axis = plt.subplots(figsize=KALIBR_PAGE)
    figure.suptitle(f"{report['camera_id']}: reprojection errors", fontsize=10)
    points = axis.scatter(residuals[:, 0], residuals[:, 1], c=times, cmap='turbo', s=3, alpha=.55,
                          linewidths=0, rasterized=True)
    limit = max(float(np.max(np.abs(residuals))), .05) * 1.05
    axis.axhline(0, color='.25', linewidth=.7)
    axis.axvline(0, color='.25', linewidth=.7)
    axis.set_xlim(-limit, limit)
    axis.set_ylim(-limit, limit)
    axis.set_aspect('equal', adjustable='box')
    axis.set_title(f"{report['camera_id']} reprojection errors")
    axis.set_xlabel('error x [pixels]')
    axis.set_ylabel('error y [pixels]')
    axis.grid(True, alpha=.25)
    colorbar = figure.colorbar(points, ax=axis, shrink=.82, pad=.04)
    colorbar.set_label('time [s]')
    figure.text(.1, .035, 'Every evaluated corner; full range, no clipping.', fontsize=6)
    figure.subplots_adjust(top=.89, bottom=.12, left=.10, right=.90)
    finish(pdf, figure)


def render(input_path, output_path):
    with input_path.open(encoding='utf-8') as stream:
        report = json.load(stream)
    if report.get('schema_version') != 3:
        raise ValueError('unsupported VI report schema')
    times, positions, rotations = motion_arrays(report)
    if len(times) == 0:
        raise ValueError('VI report has no motion samples')
    if len(report.get('imu', [])) < 2:
        raise ValueError('VI report has fewer than two IMU samples')
    imu_times, imu = imu_arrays(report)
    residuals, residual_times, held_out = residual_arrays(report)
    plt.rcParams.update({
        'font.size': 7,
        'axes.titlesize': 8,
        'axes.labelsize': 7,
        'xtick.labelsize': 6,
        'ytick.labelsize': 6,
        'figure.facecolor': 'white',
        'savefig.facecolor': 'white',
    })
    metadata = {'Title': 'Camera-IMU calibration report', 'Author': 'cam_imu_calib'}
    with PdfPages(output_path, metadata=metadata) as pdf:
        summary_page(pdf, report)
        trajectory_page(pdf, positions, rotations)
        component_page(pdf, times, positions, rotations)
        imu_rate_page(pdf, imu_times)
        vector_comparison_page(pdf, 'Comparison of predicted and measured specific force (imu0 frame)',
                               imu_times, imu['predicted_accel_m_s2'], imu['measured_accel_m_s2'], 'm/s2')
        vector_error_page(pdf, 'imu0: acceleration error', imu_times, imu['accel_residual_m_s2'], 'm/s2',
                          report['diagnostics']['accel_sample_sigma_m_s2'])
        bias_page(pdf, 'imu0: estimated accelerometer bias (imu frame)', imu_times,
                  imu['accel_bias_m_s2'], 'm/s2', report['diagnostics']['accel_random_walk'])
        vector_comparison_page(pdf, 'Comparison of predicted and measured angular velocities (imu0 frame)',
                               imu_times, imu['predicted_gyro_rad_s'], imu['measured_gyro_rad_s'], 'rad/s')
        vector_error_page(pdf, 'imu0: angular velocities error', imu_times, imu['gyro_residual_rad_s'], 'rad/s',
                          report['diagnostics']['gyro_sample_sigma_rad_s'])
        bias_page(pdf, 'imu0: estimated gyro bias (imu frame)', imu_times,
                  imu['gyro_bias_rad_s'], 'rad/s', report['diagnostics']['gyro_random_walk'])
        residual_time_page(pdf, report, residuals, residual_times, held_out)
        residual_distribution_page(pdf, report, residuals, residual_times)


def main():
    if len(sys.argv) != 3:
        raise SystemExit('usage: render_vi_report.py INPUT.json OUTPUT.pdf')
    input_path, output_path = map(Path, sys.argv[1:])
    if output_path.exists():
        raise FileExistsError(f'report output already exists: {output_path}')
    render(input_path, output_path)


if __name__ == '__main__':
    main()
