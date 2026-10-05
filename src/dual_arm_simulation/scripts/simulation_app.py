#!/usr/bin/env python3
"""Desktop physics viewer with target controls and measured simulation state."""
import argparse
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time

# EGL also supports an offscreen smoke test; Qt presents rendered RGB frames.
os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from simulation_model import Simulation, build_scene


class Viewport(QtWidgets.QLabel):
    def __init__(self, simulation):
        super().__init__()
        self.sim = simulation
        self.setMinimumSize(620, 560)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet('background: #e5ebf1; border: 1px solid #c7d2df; border-radius: 8px;')
        self.renderer = mujoco.Renderer(simulation.model, height=700, width=850)
        self.camera = mujoco.MjvCamera()
        self.options = mujoco.MjvOption()
        self.options.geomgroup[0] = 0  # Physics collision proxies hidden by default.
        self.previous = None
        self.reset_camera()

    def reset_camera(self, front=False):
        self.camera.lookat[:] = [0, 0, .23]
        self.camera.distance = .82
        self.camera.azimuth = 0 if front else 35
        self.camera.elevation = -12 if front else -20

    def draw(self):
        self.renderer.update_scene(self.sim.data, camera=self.camera, scene_option=self.options)
        rgb = self.renderer.render()
        img = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
        self.setPixmap(QtGui.QPixmap.fromImage(img).scaled(self.size(), QtCore.Qt.KeepAspectRatio,
                                                         QtCore.Qt.SmoothTransformation))

    def mousePressEvent(self, event):
        self.previous = event.pos()

    def mouseMoveEvent(self, event):
        if self.previous is None:
            return
        dx, dy = event.x() - self.previous.x(), event.y() - self.previous.y()
        self.previous = event.pos()
        pan = event.buttons() & (QtCore.Qt.RightButton | QtCore.Qt.MiddleButton) or event.modifiers() & QtCore.Qt.ShiftModifier
        action = mujoco.mjtMouse.mjMOUSE_MOVE_V if pan else mujoco.mjtMouse.mjMOUSE_ROTATE_V
        mujoco.mjv_moveCamera(self.sim.model, action, dx / self.height(), dy / self.height(),
                             self.renderer.scene, self.camera)

    def wheelEvent(self, event):
        mujoco.mjv_moveCamera(self.sim.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0,
                             .1 * event.angleDelta().y() / 120, self.renderer.scene, self.camera)

    def mouseReleaseEvent(self, event):
        self.previous = None


class SimulationWindow(QtWidgets.QMainWindow):
    def __init__(self, simulation, log_dir):
        super().__init__()
        self.sim, self.log_dir = simulation, Path(log_dir)
        self.csv_file = None
        self.csv_writer = None
        self.last_csv_time = -1
        self.setWindowTitle('Dual Arm — 디지털 트윈 물리 시뮬레이션')
        self.resize(1580, 900)
        self.setStyleSheet('QMainWindow, QWidget { background: #f4f7fa; color: #1c3046; font-size: 13px; }'
                          'QPushButton { background: #e1e9f2; padding: 7px 12px; border: 1px solid #bbcbdc; border-radius: 5px; }'
                          'QPushButton:checked { background: #216b9f; color: white; }'
                          'QGroupBox { font-weight: bold; border: 1px solid #c6d4e1; border-radius: 6px; margin-top: 10px; padding-top: 10px; }')
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        title = QtWidgets.QLabel('DUAL ARM  /  디지털 트윈 물리 시뮬레이션')
        title.setStyleSheet('font-size: 23px; font-weight: bold; padding: 5px;')
        layout.addWidget(title)
        subtitle = QtWidgets.QLabel('SIMULATION · 중력 보상 + 관절별 PID · 토크 제한 적용 · 실기 연결 없음')
        subtitle.setStyleSheet('color: #256e96; padding-left: 5px;')
        layout.addWidget(subtitle)
        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)
        self.pause_button = QtWidgets.QPushButton('일시정지')
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(self.pause)
        controls.addWidget(self.pause_button)
        self.torque_button = QtWidgets.QPushButton('가상 모터 ON')
        self.torque_button.setCheckable(True)
        self.torque_button.setChecked(True)
        self.torque_button.toggled.connect(self.torque)
        controls.addWidget(self.torque_button)
        for name, callback in (('초기화', self.reset), ('영점 목표', self.zero_targets),
                               ('어깨 90°', self.shoulder_targets), ('팔 굽히기', self.bend_targets),
                               ('정면', lambda: self.viewport.reset_camera(True)),
                               ('입체 시점', lambda: self.viewport.reset_camera(False))):
            button = QtWidgets.QPushButton(name)
            button.clicked.connect(callback)
            controls.addWidget(button)
        collision_button = QtWidgets.QPushButton('충돌 형상')
        collision_button.setCheckable(True)
        collision_button.toggled.connect(lambda on: self.toggle_collisions(on))
        controls.addWidget(collision_button)
        self.record_button = QtWidgets.QPushButton('CSV 기록')
        self.record_button.setCheckable(True)
        self.record_button.toggled.connect(self.record)
        controls.addWidget(self.record_button)
        controls.addStretch()
        content = QtWidgets.QHBoxLayout()
        layout.addLayout(content, 1)
        self.viewport = Viewport(simulation)
        content.addWidget(self.viewport, 1)
        panel = QtWidgets.QWidget()
        panel.setMinimumWidth(730)
        panel_layout = QtWidgets.QVBoxLayout(panel)
        content.addWidget(panel)
        self.spins, self.sliders, self.cells = [], [], []
        for side, caption in (('left', '왼팔'), ('right', '오른팔')):
            group = QtWidgets.QGroupBox(caption)
            grid = QtWidgets.QGridLayout(group)
            for col, text in enumerate(('관절', '목표 조작', '목표 °', '현재 °', '오차 °', '속도 °/s', '토크 / 상한 N·m', '상태')):
                grid.addWidget(QtWidgets.QLabel(text), 0, col)
            row = 1
            for index, name in enumerate(self.sim.names):
                if not name.startswith(side + '_'):
                    continue
                part = name.removeprefix(side + '_').removesuffix('_joint')
                caption = {'shoulder_pitch': '어깨 pitch', 'shoulder_roll': '어깨 roll',
                           'elbow_pitch': '팔꿈치', 'wrist_roll': '손목 roll',
                           'inner_finger': '집게 안쪽', 'outer_finger': '집게 바깥'}[part]
                grid.addWidget(QtWidgets.QLabel(caption), row, 0)
                slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
                slider.setRange(round(np.degrees(self.sim.lower[index]) * 10), round(np.degrees(self.sim.upper[index]) * 10))
                slider.setMinimumWidth(95)
                slider.setMaximumWidth(130)
                spin = QtWidgets.QDoubleSpinBox()
                spin.setDecimals(1)
                spin.setRange(float(np.degrees(self.sim.lower[index])), float(np.degrees(self.sim.upper[index])))
                spin.setSingleStep(.5)
                spin.setFixedWidth(72)
                slider.valueChanged.connect(lambda value, i=index: self.target_changed(i, value / 10))
                spin.valueChanged.connect(lambda value, i=index: self.target_changed(i, value))
                self.sliders.append(slider)
                self.spins.append(spin)
                grid.addWidget(slider, row, 1)
                grid.addWidget(spin, row, 2)
                cells = [QtWidgets.QLabel('--') for _ in range(5)]
                for col, cell in enumerate(cells, 3):
                    cell.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                    grid.addWidget(cell, row, col)
                self.cells.append(cells)
                row += 1
            panel_layout.addWidget(group)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        panel_layout.addWidget(self.status)
        explanation = QtWidgets.QLabel('현재 토크와 관절별 상한을 함께 표시합니다.\n'
            '중력 보상 토크도 같은 상한 안에서 출력됩니다.\n'
            '도달: 오차 0.5° 이내 / 속도 1°/s 이내\n'
            '영점: 실제 모터 180° ↔ 시뮬레이션 관절 0°\n'
            '전압·온도·전류는 현재 모델에서 계산하지 않습니다.\n\n'
            '토크 상한과 PID는 시뮬레이션용 설정입니다.\n'
            '실기 Torque Limit 설정은 변경하지 않습니다.\n'
            '질량·관성·서보 특성은 실측 보정 전입니다.')
        explanation.setStyleSheet('color: #52667d; padding: 10px;')
        panel_layout.addWidget(explanation)
        panel_layout.addStretch()
        layout.addWidget(QtWidgets.QLabel('화면: 왼쪽 드래그 회전 · 오른쪽/Shift 드래그 이동 · 휠 확대/축소  |  주황 블록: 중력·접촉 확인용 25 g 물체'))
        self.previous_wall = time.monotonic()
        self.previous_sim = float(self.sim.data.time)
        self.accumulator = 0.0
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)
        self.sync_targets()
        self.refresh()

    def target_changed(self, index, degrees):
        values = self.sim.targets.copy()
        values[index] = np.radians(degrees)
        self.sim.set_targets(values)
        self.sync_targets()

    def sync_targets(self):
        for i, value in enumerate(np.degrees(self.sim.targets)):
            for widget in (self.sliders[i], self.spins[i]):
                widget.blockSignals(True)
            self.sliders[i].setValue(round(value * 10))
            self.spins[i].setValue(float(value))
            for widget in (self.sliders[i], self.spins[i]):
                widget.blockSignals(False)

    def zero_targets(self):
        self.sim.set_targets(np.zeros(len(self.sim.names)))
        self.sync_targets()

    def bend_targets(self):
        values = [20 if 'shoulder_pitch' in n else 15 if 'shoulder_roll' in n
                  else 40 if 'elbow_pitch' in n else 0 for n in self.sim.names]
        self.sim.set_targets(np.radians(values))
        self.sync_targets()

    def shoulder_targets(self):
        values = np.zeros(len(self.sim.names))
        for i, name in enumerate(self.sim.names):
            if 'shoulder_pitch' in name:
                values[i] = np.pi / 2
        self.sim.set_targets(values)
        self.sync_targets()

    def pause(self, on):
        self.sim.paused = on
        self.accumulator = 0.0
        self.pause_button.setText('계속 실행' if on else '일시정지')

    def torque(self, on):
        try:
            self.sim.set_enabled(on)
        except RuntimeError:
            self.torque_button.blockSignals(True)
            self.torque_button.setChecked(False)
            self.torque_button.blockSignals(False)
            on = False
        self.torque_button.setText('가상 모터 ON' if on else '가상 모터 OFF')
        self.sync_targets()

    def reset(self):
        # A reset starts a new timeline; finish an active CSV instead of mixing times.
        self.record_button.setChecked(False)
        self.sim.reset()
        self.torque_button.setChecked(True)
        self.previous_sim = 0.0
        self.accumulator = 0.0
        self.sync_targets()

    def toggle_collisions(self, on):
        self.viewport.options.geomgroup[0] = int(on)

    def record(self, on):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file, self.csv_writer = None, None
        if on:
            try:
                self.log_dir.mkdir(parents=True, exist_ok=True)
                path = self.log_dir / ('simulation_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '.csv')
                self.csv_file = path.open('x', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(('sim_time_s', 'joint', 'target_deg', 'position_deg', 'velocity_deg_s',
                                          'actuator_torque_nm', 'torque_cap_nm', 'saturated', 'contacts', 'motor_enabled',
                                          'error_deg', 'bias_feedforward_nm', 'integral_torque_nm'))
                self.last_csv_time = -1
                self.record_button.setText('기록 중')
                self.statusBar().showMessage(str(path))
            except OSError as exc:
                self.record_button.setChecked(False)
                self.statusBar().showMessage(str(exc))
        else:
            self.record_button.setText('CSV 기록')

    def tick(self):
        now = time.monotonic()
        elapsed = now - self.previous_wall
        self.previous_wall = now
        if not self.sim.paused and not self.sim.fault:
            self.accumulator += min(elapsed, .1)
            count = int(self.accumulator / self.sim.model.opt.timestep)
            self.accumulator -= count * self.sim.model.opt.timestep
            try:
                self.sim.step(count)
            except RuntimeError:
                self.torque_button.setChecked(False)
        realtime = (self.sim.data.time - self.previous_sim) / max(elapsed, 1e-9)
        self.previous_sim = float(self.sim.data.time)
        self.refresh(realtime)

    def refresh(self, realtime=0):
        sample = self.sim.snapshot()
        control = self.sim.manifest['config']['controller']
        for i, cells in enumerate(self.cells):
            error_deg = np.degrees(sample['error'][i])
            velocity_deg_s = np.degrees(sample['velocity'][i])
            reached = (abs(error_deg) <= control['tracking_tolerance_deg'] and
                       abs(velocity_deg_s) <= control['settled_velocity_deg_s'])
            state = ('오류' if self.sim.fault else 'OFF' if not self.sim.enabled else
                     '토크 제한' if sample['saturated'][i] else '도달' if reached else '이동 중')
            texts = (f'{np.degrees(sample["position"][i]):.1f}', f'{error_deg:+.2f}', f'{velocity_deg_s:.1f}',
                     f'{sample["effort"][i]:+.3f} / {self.sim.caps[i]:.2f}', state)
            for cell, text in zip(cells, texts):
                cell.setText(text)
            cells[-1].setStyleSheet('color: #b85d13; font-weight: bold;' if sample['saturated'][i] else '')
        state = self.sim.fault or ('일시정지' if self.sim.paused else '실행 중')
        self.status.setText(f'{state}  |  시뮬레이션 {sample["time"]:.2f} s\n'
                            f'실시간 비율 {realtime:.2f}×  |  접촉점 {sample["contacts"]}개')
        if self.csv_writer and sample['time'] - self.last_csv_time >= .1:
            for i, name in enumerate(self.sim.names):
                self.csv_writer.writerow((sample['time'], name, np.degrees(sample['target'][i]),
                    np.degrees(sample['position'][i]), np.degrees(sample['velocity'][i]), sample['effort'][i],
                    self.sim.caps[i], bool(sample['saturated'][i]), sample['contacts'], self.sim.enabled,
                    np.degrees(sample['error'][i]), sample['feedforward'][i], sample['integral_torque'][i]))
            self.csv_file.flush()
            self.last_csv_time = sample['time']
        self.viewport.draw()

    def closeEvent(self, event):
        self.timer.stop()
        self.record(False)
        self.viewport.renderer.close()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description='Dual-arm physics simulation and monitor')
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--description-dir', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--headless', action='store_true', help='run physics without opening a window')
    parser.add_argument('--seconds', type=float, default=3)
    parser.add_argument('--pose', choices=('neutral', 'shoulders90'), default='neutral',
                        help='initial target pose; the robot moves there through physics')
    parser.add_argument('--screenshot', type=Path, help='save the complete window; with --headless use offscreen Qt')
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    scene, manifest = build_scene(args.urdf, args.description_dir, args.config, args.output_dir)
    sim = Simulation(scene, manifest)
    if args.pose == 'shoulders90':
        target = np.zeros(len(sim.names))
        for i, name in enumerate(sim.names):
            if 'shoulder_pitch' in name:
                target[i] = np.pi / 2
        sim.set_targets(target)
    if args.headless:
        sim.step(round(args.seconds / sim.model.opt.timestep))
        sample = sim.snapshot()
        print(json.dumps({'sim_time_s': sim.data.time, 'joints': len(sim.names), 'contacts': sim.data.ncon,
                          'max_abs_torque_nm': float(np.max(np.abs(sim.snapshot()['effort']))),
                          'max_tracking_error_deg': float(np.max(np.abs(np.degrees(sample['error'])))),
                          'position_deg': dict(zip(sim.names, np.degrees(sample['position']))),
                          'urdf_sha256': manifest['urdf_sha256']}))
        if not args.screenshot:
            return
        os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setFont(QtGui.QFont('Noto Sans CJK KR', 10))
    window = SimulationWindow(sim, args.output_dir / 'records')
    window.show()
    if args.screenshot:
        args.screenshot.parent.mkdir(parents=True, exist_ok=True)

        def capture():
            if not window.grab().save(str(args.screenshot)):
                raise RuntimeError('Screenshot save failed')
            if args.headless:
                window.close()

        QtCore.QTimer.singleShot(800, capture)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
