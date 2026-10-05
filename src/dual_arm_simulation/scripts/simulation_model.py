"""Build physics from the current URDF, then drive it with torque-limited servos.

No hardware connection, device writes, ROS joint-state publishers or fake sensors.
"""
from pathlib import Path
import hashlib
import json
import math
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml


def controller_settings(config, names):
    """Validate a complete per-joint simulation profile before creating actuators."""
    control = config['controller']
    if type(control['bias_compensation']) is not bool:
        raise ValueError('bias_compensation must be a boolean')
    for key in ('tracking_tolerance_deg', 'settled_velocity_deg_s'):
        value = control[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid {key}')
    profiles = control['profiles']
    roles = {n.removeprefix('left_').removeprefix('right_').removesuffix('_joint') for n in names}
    if set(profiles) != roles:
        raise ValueError('Controller profiles must cover exactly the robot joint roles')
    result = {}
    for name in names:
        role = name.removeprefix('left_').removeprefix('right_').removesuffix('_joint')
        values = profiles[role]
        if set(values) != {'kp', 'kd', 'ki', 'integral_limit_nm', 'effort_limit_nm'}:
            raise ValueError(f'Invalid controller keys for {role}')
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in values.values()):
            raise ValueError(f'Controller values must be finite and positive: {role}')
        if values['integral_limit_nm'] > values['effort_limit_nm']:
            raise ValueError(f'Integral limit exceeds torque cap: {role}')
        result[name] = dict(values)
    return result


def build_scene(urdf_path, description_dir, config_path, output_dir):
    description_dir, output_dir = Path(description_dir).resolve(), Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    urdf_text = Path(urdf_path).read_text()
    urdf = ET.fromstring(urdf_text)
    config = yaml.safe_load(Path(config_path).read_text())
    for key in ('timestep',):
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f'{key} must be positive and finite')
    if not 0 < config['timestep'] <= .005:
        raise ValueError('timestep must be at most 0.005 s')
    for key in ('gravity', 'contact_friction'):
        if len(config[key]) != 3 or not all(math.isfinite(v) for v in config[key]):
            raise ValueError(f'Invalid {key}')
    if any(v < 0 for v in config['contact_friction']):
        raise ValueError('Negative friction')
    profiles = controller_settings(config, [j.get('name') for j in urdf.findall("joint[@type='revolute']")])
    calibration = yaml.safe_load((description_dir / 'config/motor_calibration.yaml').read_text())
    motor_ids = calibration['motor_ids']
    if (set(motor_ids) != set(profiles) or len(set(motor_ids.values())) != len(profiles)
            or any(type(value) is not int or not 0 <= value <= 253 for value in motor_ids.values())):
        raise ValueError('Motor IDs must cover each robot joint exactly once with unique IDs in 0..253')
    for mesh in urdf.findall('.//mesh'):
        prefix = 'package://dual_arm_description/'
        filename = mesh.get('filename')
        if not filename.startswith(prefix):
            raise ValueError(f'Unsupported mesh URI: {filename}')
        mesh.set('filename', str(description_dir / filename[len(prefix):]))
    extra = ET.SubElement(urdf, 'mujoco')
    ET.SubElement(extra, 'compiler', discardvisual='false', fusestatic='false', strippath='false')
    imported = mujoco.MjModel.from_xml_string(ET.tostring(urdf, encoding='unicode'))
    # Use MuJoCo's own URDF importer to preserve fixed-link transforms and inertia.
    with tempfile.TemporaryDirectory(dir=output_dir) as temporary:
        converted = Path(temporary) / 'import.xml'
        mujoco.mj_saveLastXML(str(converted), imported)
        scene = ET.parse(converted).getroot()
    ET.SubElement(scene, 'option', timestep=str(config['timestep']),
                  gravity=' '.join(map(str, config['gravity'])), integrator='implicitfast',
                  iterations='60', tolerance='1e-10')
    visual = ET.SubElement(scene, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='900')
    ET.SubElement(visual, 'headlight', ambient='.25 .25 .25', diffuse='.35 .35 .35')
    ET.SubElement(visual, 'rgba', haze='.93 .95 .97 1')
    ET.SubElement(scene, 'statistic', center='0 0 .24', extent='.6')
    world = scene.find('worldbody')
    asset = scene.find('asset')
    ET.SubElement(asset, 'texture', type='skybox', builtin='gradient', width='256', height='1536',
                  rgb1='.87 .92 .97', rgb2='.98 .99 1')
    ET.SubElement(asset, 'texture', name='floor_texture', type='2d', builtin='checker',
                  width='512', height='512', rgb1='.84 .87 .9', rgb2='.93 .95 .97')
    ET.SubElement(asset, 'material', name='floor_material', texture='floor_texture',
                  texrepeat='16 16', texuniform='true', reflectance='.08')
    ET.SubElement(world, 'light', pos='.5 -.5 1.5', dir='-.2 .2 -1', directional='true',
                  diffuse='.45 .45 .45', specular='.1 .1 .1')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='1 1 .01', group='2',
                  material='floor_material', friction=' '.join(map(str, config['contact_friction'])))
    # A fixed support represents the bench fixture; robot crossbar remains unchanged.
    crossbar = world.find(".//body[@name='crossbar_link']")
    mount_bottom = float(crossbar.get('pos').split()[2]) - .01
    ET.SubElement(world, 'geom', name='fixture', type='box', group='2',
                  pos=f'0 0 {mount_bottom / 2}', size=f'.016 .020 {mount_bottom / 2}',
                  rgba='.28 .32 .36 1')
    ET.SubElement(world, 'geom', name='fixture_base', type='box', group='2',
                  pos='0 0 .008', size='.065 .06 .008', rgba='.28 .32 .36 1')
    # A free object makes gravity/contact observable without changing robot geometry.
    block = ET.SubElement(world, 'body', name='test_block', pos='.13 0 .06')
    ET.SubElement(block, 'freejoint', name='test_block_free')
    ET.SubElement(block, 'geom', name='test_block_geom', type='box', size='.018 .018 .018',
                  mass='.025', group='2', rgba='.93 .48 .14 1',
                  friction=' '.join(map(str, config['contact_friction'])))
    for geom in world.findall('.//geom'):
        # URDF visual meshes are group 1 with contacts disabled by the importer.
        if geom.get('group', '0') == '0':
            geom.set('rgba', '.1 .6 .85 .3')
            geom.set('friction', ' '.join(map(str, config['contact_friction'])))
    actuators = ET.SubElement(scene, 'actuator')
    joints = []
    for source in urdf.findall("joint[@type='revolute']"):
        name = source.get('name')
        limit = source.find('limit')
        lower, upper, urdf_effort = (float(limit.get(k)) for k in ('lower', 'upper', 'effort'))
        if not all(math.isfinite(x) for x in (lower, upper, urdf_effort)) or lower >= upper or urdf_effort <= 0:
            raise ValueError(f'Invalid limits: {name}')
        effort = profiles[name]['effort_limit_nm']
        joint = world.find(f".//joint[@name='{name}']")
        joint.set('range', f'{lower} {upper}')
        joint.set('limited', 'true')
        joint.set('actuatorfrclimited', 'true')
        joint.set('actuatorfrcrange', f'{-effort} {effort}')
        joint.set('solreflimit', '.004 1')
        ET.SubElement(actuators, 'motor', name=name + '_servo', joint=name, gear='1',
                      ctrllimited='true', ctrlrange=f'{-effort} {effort}',
                      forcelimited='true', forcerange=f'{-effort} {effort}')
        joints.append({'name': name, 'motor_id': motor_ids[name], 'lower': lower, 'upper': upper, 'effort': effort,
                       'urdf_effort': urdf_effort, 'controller': profiles[name]})
    target = output_dir / 'dual_arm_scene.xml'
    ET.indent(scene)
    ET.ElementTree(scene).write(target, encoding='utf-8', xml_declaration=True)
    manifest = {'urdf_sha256': hashlib.sha256(urdf_text.encode()).hexdigest(),
                'urdf': str(Path(urdf_path).resolve()), 'engine': mujoco.__version__,
                'controller_sha256': hashlib.sha256(Path(config_path).read_bytes()).hexdigest(),
                'joints': joints, 'config': config,
                'limitations': 'Simulation-only caps and bias-compensated PID; not calibrated hardware torque or servo dynamics.'}
    (output_dir / 'model_manifest.json').write_text(json.dumps(manifest, indent=2))
    return target, manifest


class Simulation:
    def __init__(self, scene_path, manifest):
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        self.manifest = manifest
        self.names = [j['name'] for j in manifest['joints']]
        # Bus IDs are labels; self.ids below remains MuJoCo's internal joint indices.
        self.motor_ids = [j['motor_id'] for j in manifest['joints']]
        self.ids = np.array([self.model.joint(n).id for n in self.names])
        self.qindices = self.model.jnt_qposadr[self.ids]
        self.vindices = self.model.jnt_dofadr[self.ids]
        self.aindices = np.array([self.model.actuator(n + '_servo').id for n in self.names])
        self.lower = np.array([j['lower'] for j in manifest['joints']])
        self.upper = np.array([j['upper'] for j in manifest['joints']])
        self.caps = np.array([j['effort'] for j in manifest['joints']])
        self.kp = np.array([j['controller']['kp'] for j in manifest['joints']])
        self.kd = np.array([j['controller']['kd'] for j in manifest['joints']])
        self.ki = np.array([j['controller']['ki'] for j in manifest['joints']])
        self.integral_limit = np.array([j['controller']['integral_limit_nm'] for j in manifest['joints']])
        self.integral_torque = np.zeros(len(self.names))
        self.feedforward = np.zeros(len(self.names))
        self.targets = np.zeros(len(self.names))
        self.enabled = True
        self.paused = False
        self.fault = ''
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.targets[:] = 0
        self.integral_torque[:] = 0
        self.feedforward[:] = 0
        self.enabled, self.fault = True, ''
        mujoco.mj_forward(self.model, self.data)

    def set_targets(self, values):
        values = np.asarray(values, dtype=float)
        if values.shape != self.targets.shape or not np.all(np.isfinite(values)):
            raise ValueError('Supply one finite target per joint')
        bounded = np.clip(values, self.lower, self.upper)
        self.integral_torque[np.abs(bounded - self.targets) > 1e-12] = 0
        self.targets[:] = bounded

    def set_enabled(self, enabled):
        if enabled and self.fault:
            raise RuntimeError('Reset required after a simulation fault')
        if enabled and not self.enabled:
            self.set_targets(self.data.qpos[self.qindices])
        self.enabled = bool(enabled)
        self.integral_torque[:] = 0
        if not self.enabled:
            self.data.ctrl[:] = 0
            self.feedforward[:] = 0
            mujoco.mj_forward(self.model, self.data)

    def _check_physics(self):
        if (not np.all(np.isfinite(self.data.qpos)) or
                not np.all(np.isfinite(self.data.qvel)) or
                any(w.number for w in self.data.warning)):
            self.fault = 'Physics solver warning: reset required'
            self.enabled = False
            self.data.ctrl[:] = 0
            self.integral_torque[:] = 0
            self.feedforward[:] = 0
            raise RuntimeError(self.fault)

    def step(self, count=1):
        if self.paused or self.fault:
            return
        bias_compensation = self.manifest['config']['controller']['bias_compensation']
        for _ in range(count):
            # Update dynamics at the current state BEFORE feedback computation.
            # Split stepping preserves the selected implicitfast integrator.
            mujoco.mj_step1(self.model, self.data)
            self._check_physics()
            if self.enabled:
                error = self.targets - self.data.qpos[self.qindices]
                self.feedforward[:] = self.data.qfrc_bias[self.vindices] if bias_compensation else 0
                base = self.feedforward + self.kp * error - self.kd * self.data.qvel[self.vindices]
                proposed = np.clip(self.integral_torque + self.ki * error * self.model.opt.timestep,
                                   -self.integral_limit, self.integral_limit)
                requested = base + proposed
                # Conditional anti-windup: integrate only while unsaturated or
                # when the error unwinds saturation. All terms share one cap.
                integrate = ((np.abs(requested) <= self.caps) |
                             ((requested > self.caps) & (error < 0)) |
                             ((requested < -self.caps) & (error > 0)))
                self.integral_torque[:] = np.where(integrate, proposed, self.integral_torque)
                self.data.ctrl[self.aindices] = np.clip(base + self.integral_torque, -self.caps, self.caps)
            else:
                self.data.ctrl[self.aindices] = 0
            mujoco.mj_step2(self.model, self.data)
            self._check_physics()

    def snapshot(self):
        q, velocity = self.data.qpos[self.qindices].copy(), self.data.qvel[self.vindices].copy()
        torque = self.data.actuator_force[self.aindices].copy()
        return {'time': float(self.data.time), 'position': q, 'velocity': velocity,
                'target': self.targets.copy(), 'effort': torque,
                'error': self.targets - q, 'feedforward': self.feedforward.copy(),
                'integral_torque': self.integral_torque.copy(),
                'saturated': np.abs(torque) >= self.caps * .999,
                'contacts': int(self.data.ncon)}
