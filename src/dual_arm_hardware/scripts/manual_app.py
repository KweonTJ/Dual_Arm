#!/usr/bin/env python3
"""Native desktop controls for actual AX-12A absolute motor angles."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys

from PyQt5 import QtCore, QtGui, QtWidgets
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/dual_arm_simulation/scripts'))
from twin_bridge import TwinBridge
from manual_control import create_controller, absolute_targets
from ax12_guard import TICKS_PER_DEGREE


class ManualWindow(QtWidgets.QMainWindow):
    def __init__(self, config, factory, log_dir, mode='real', status_file=None):
        super().__init__()
        self.names = tuple(config['motors'])
        self.ids = [config['motors'][name]['id'] for name in self.names]
        self.factory, self.log_dir, self.mode = factory, Path(log_dir), mode
        self.status_file = status_file
        self.bridge = self.new_bridge()
        self.seed_token = 0
        self.targets = {name: 0. for name in self.names}
        self.spins, self.sliders, self.cells = {}, {}, {}
        self.state = None
        self.setWindowTitle('양팔 실기 모터 제어 — AX-12A' if mode=='real' else '양팔 모터 제어 — MOCK')
        self.resize(1540, 720)
        self.setStyleSheet('QWidget { background:#f4f7fa; color:#19334c; font-size:13px; }'
                          'QPushButton { padding:9px; background:#e1eaf3; border:1px solid #b9ccdd; border-radius:5px; }'
                          'QGroupBox { font-weight:bold; border:1px solid #b9ccdd; border-radius:7px; margin-top:12px; padding-top:15px; }')
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        title = QtWidgets.QLabel('AX-12A  /  실기 모터 직접 제어' if mode=='real' else 'AX-12A  /  MOCK · 실제 장치 없음')
        title.setStyleSheet('font-size:24px; font-weight:bold; padding:5px;')
        layout.addWidget(title)
        layout.addWidget(QtWidgets.QLabel('모터 절대각 0~300° · URDF 관절각과 별도 · 토크 ON 시 현재 위치 유지'))
        layout.addWidget(QtWidgets.QLabel(f'{config["port"]}  |  {config["baudrate"]:,} baud  |  보호·속도 설정 유지'))
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.connect_button = QtWidgets.QPushButton('연결 / 재연결')
        self.enable_button = QtWidgets.QPushButton('전체 토크 ON · 현재 위치 유지')
        self.hold_button = QtWidgets.QPushButton('현재 위치 유지')
        self.off_button = QtWidgets.QPushButton('전체 토크 OFF')
        self.stop_button = QtWidgets.QPushButton('긴급 정지')
        self.stop_button.setStyleSheet('background:#b93535; color:white; font-weight:bold; padding:10px;')
        self.connect_button.clicked.connect(self.connect_bus)
        for button, operation in ((self.enable_button,'enable'),(self.hold_button,'hold'),
                                  (self.off_button,'disable'),(self.stop_button,'estop')):
            button.clicked.connect(lambda _=False,op=operation:self.action(op))
        for button in (self.connect_button,self.enable_button,self.hold_button,self.off_button,self.stop_button):
            row.addWidget(button)
        content = QtWidgets.QHBoxLayout(); layout.addLayout(content)
        roles = dict(shoulder_pitch='어깨 pitch',shoulder_roll='어깨 roll',elbow_pitch='팔꿈치',
                     wrist_roll='손목 roll',gripper='집게')
        for side, caption in (('right','오른팔 · 정면 왼쪽'),('left','왼팔 · 정면 오른쪽')):
            group = QtWidgets.QGroupBox(caption); grid = QtWidgets.QGridLayout(group); content.addWidget(group)
            grid.setHorizontalSpacing(10)
            for col,width in ((0,70),(2,94),(3,50),(4,34),(5,40),(6,40),(7,36)):
                grid.setColumnMinimumWidth(col,width)
            for col, text in enumerate(('모터','목표 조작','목표 °','현재 °','raw','전압 V','온도 °C','토크')):
                grid.addWidget(QtWidgets.QLabel(text),0,col)
            for index, name in enumerate(n for n in self.names if n.startswith(side+'_')):
                role = name.removeprefix(side+'_').removesuffix('_joint')
                grid.addWidget(QtWidgets.QLabel(f'ID {config["motors"][name]["id"]:02d}\n{roles[role]}'),index+1,0)
                slider = QtWidgets.QSlider(QtCore.Qt.Horizontal); slider.setRange(0,1023)
                slider.setMinimumWidth(110)
                spin = QtWidgets.QDoubleSpinBox(); spin.setRange(0,300); spin.setDecimals(2)
                spin.setSingleStep(1/TICKS_PER_DEGREE); spin.setSuffix('°'); spin.setMinimumWidth(90)
                slider.valueChanged.connect(lambda raw,n=name:self.target_changed(n,raw/TICKS_PER_DEGREE))
                spin.valueChanged.connect(lambda degrees,n=name:self.target_changed(n,degrees))
                self.spins[name], self.sliders[name] = spin, slider
                grid.addWidget(slider,index+1,1); grid.addWidget(spin,index+1,2)
                cells = [QtWidgets.QLabel('--') for _ in range(5)]
                for col, cell in enumerate(cells,3):
                    cell.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter)
                    grid.addWidget(cell,index+1,col)
                self.cells[name] = cells
        self.status = QtWidgets.QLabel(); self.status.setWordWrap(True); layout.addWidget(self.status)
        layout.addWidget(QtWidgets.QLabel('연결 → 현재각 확인 → 전체 토크 ON → 슬라이더 조작\n'
                                         '토크 OFF 또는 창 종료 시 팔의 지지력이 사라집니다.'))
        layout.addStretch()
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(50)
        self.tick()

    def new_bridge(self):
        return TwinBridge(self.names,self.ids,self.factory,self.mode,self.log_dir)

    def connect_bus(self):
        if self.bridge.snapshot()['phase'] in ('FAULT','CLOSED'):
            errors = self.bridge.close()
            if errors:
                self.statusBar().showMessage('OFF 미확인: '+'; '.join(errors)); return
            self.bridge = self.new_bridge(); self.seed_token = 0
        self.action('connect')

    def action(self, operation):
        try:
            self.bridge.request(operation)
            self.tick()
        except RuntimeError as exc:
            self.statusBar().showMessage(str(exc))

    def sync_targets(self):
        for name, degrees in self.targets.items():
            spin, slider = self.spins[name], self.sliders[name]
            spin.blockSignals(True); slider.blockSignals(True)
            spin.setValue(degrees); slider.setValue(round(degrees*TICKS_PER_DEGREE))
            spin.blockSignals(False); slider.blockSignals(False)

    def target_changed(self, name, degrees):
        if not self.state or self.state['phase']!='ACTIVE': return
        self.targets[name] = float(degrees)
        self.sync_targets()
        try:
            self.bridge.submit_targets(absolute_targets(self.targets,self.state['feedback']))
        except (ValueError,KeyError) as exc:
            self.statusBar().showMessage(str(exc))

    def tick(self):
        self.bridge.pulse(); state = self.bridge.snapshot(); self.state = state
        phase, feedback = state['phase'], state['feedback']
        valid = feedback.get('sample_valid',False)
        samples = feedback.get('samples',{})
        if valid:
            for name in self.names:
                lo, hi = feedback['motor_bounds_raw'][name]
                spin, slider = self.spins[name], self.sliders[name]
                spin.blockSignals(True); slider.blockSignals(True)
                spin.setRange(lo/TICKS_PER_DEGREE,hi/TICKS_PER_DEGREE); slider.setRange(lo,hi)
                spin.blockSignals(False); slider.blockSignals(False)
            if phase=='TORQUE_OFF' or (phase=='AWAITING_SEED' and state['seed_token']!=self.seed_token):
                self.targets = {n:samples[n]['motor_position_deg'] for n in self.names}
                self.sync_targets()
                if phase=='AWAITING_SEED':
                    self.bridge.acknowledge_seed(state['seed_token']); self.seed_token = state['seed_token']
        for name, cells in self.cells.items():
            sample = samples.get(name) if valid else None
            values = ([f'{sample["motor_position_deg"]:.2f}',str(sample['position_raw']),
                       f'{sample["voltage_v"]:.1f}',str(sample['temperature_c']),
                       'ON' if sample['torque_enabled'] else 'OFF'] if sample else ['--']*5)
            for cell,value in zip(cells,values): cell.setText(value)
        labels = dict(DISCONNECTED='미연결',CONNECTING='연결 중',TORQUE_OFF='연결됨 · 전체 토크 OFF',
                      ENABLING='현재 위치 유지 준비',AWAITING_SEED='목표를 현재 위치에 맞추는 중',
                      ACTIVE='실기 제어 중',HOLDING='현재 위치 유지 요청 중',DISABLING='토크 OFF 요청 중',
                      STOPPING='정지 중',FAULT='오류 · 재연결 필요',CLOSED='연결 종료')
        prefix = '실기' if self.mode=='real' else 'MOCK'
        self.status.setText(f'{prefix}: {labels[phase]}' + ('\n'+state['error'] if state['error'] else '') +
                            ('\nOFF 미확인: '+'; '.join(state['torque_off_unconfirmed']) if state['torque_off_unconfirmed'] else ''))
        self.connect_button.setEnabled(phase in ('DISCONNECTED','FAULT','CLOSED'))
        self.enable_button.setEnabled(phase=='TORQUE_OFF' and valid)
        self.hold_button.setEnabled(phase=='ACTIVE' and valid)
        self.off_button.setEnabled(phase in ('ACTIVE','AWAITING_SEED','HOLDING'))
        self.stop_button.setEnabled(phase in ('TORQUE_OFF','ENABLING','ACTIVE','AWAITING_SEED','HOLDING'))
        for widget in (*self.spins.values(),*self.sliders.values()): widget.setEnabled(phase=='ACTIVE' and valid)
        if self.status_file:
            summary = dict(phase=phase,error=state['error'],sample_valid=valid,mode=self.mode,
                           motor_ids=dict(zip(self.names,self.ids)),targets_deg=self.targets,
                           samples=samples,torque_off_unconfirmed=state['torque_off_unconfirmed'],
                           motor_reference_raw=feedback.get('motor_reference_raw',{}))
            temporary = self.status_file.with_suffix('.tmp')
            temporary.write_text(json.dumps(summary,indent=2),encoding='utf-8')
            temporary.replace(self.status_file)

    def closeEvent(self, event):
        self.timer.stop(); errors = self.bridge.close()
        if errors:
            print('Torque OFF unconfirmed: '+'; '.join(errors),file=sys.stderr,flush=True)
        if self.status_file:
            self.status_file.write_text(json.dumps(dict(phase='FAULT' if errors else 'CLOSED',
                mode=self.mode,sample_valid=False,samples={},torque_off_unconfirmed=errors)),encoding='utf-8')
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mock',action='store_true')
    parser.add_argument('--connect',action='store_true',help='Connect read-only; never auto-enable torque')
    parser.add_argument('--config',type=Path,default=ROOT/'src/dual_arm_hardware/config/hardware.yaml')
    parser.add_argument('--screenshot',type=Path)
    parser.add_argument('--status-file',type=Path)
    parser.add_argument('--headless',action='store_true')
    args = parser.parse_args()
    if args.headless:
        if not args.mock: parser.error('--headless is only supported with --mock')
        os.environ['QT_QPA_PLATFORM']='offscreen'
    config = yaml.safe_load(args.config.read_text())
    def factory():
        from mock_bus import MockBus
        return create_controller(args.config,ROOT/'src/dual_arm_description/config',
                                 ROOT/'src/dual_arm_hardware/config/lowlevel.yaml',
                                 (lambda motors:MockBus(motors)) if args.mock else None)
    log_dir = ROOT/'log/manual_control'; log_dir.mkdir(parents=True,exist_ok=True)
    if args.status_file: args.status_file.parent.mkdir(parents=True,exist_ok=True)
    app = QtWidgets.QApplication(sys.argv[:1]); app.setFont(QtGui.QFont('Noto Sans CJK KR',10))
    window = ManualWindow(config,factory,log_dir,'mock' if args.mock else 'real',args.status_file)
    window.show()
    if args.connect: QtCore.QTimer.singleShot(0,window.connect_bus)
    if args.screenshot:
        args.screenshot.parent.mkdir(parents=True,exist_ok=True)
        def capture():
            if not window.grab().save(str(args.screenshot)): raise RuntimeError('Screenshot failed')
            if args.headless: window.close()
        QtCore.QTimer.singleShot(1800,capture)
    elif args.headless: QtCore.QTimer.singleShot(1800,window.close)
    return app.exec_()


if __name__=='__main__': raise SystemExit(main())
