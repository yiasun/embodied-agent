"""Bounded Stage 6.2 profile; all coordinates derived from immutable geometry."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from .safety import (ObstacleError, site_profile, bounds_for, point_box_distance,
                     obstacle_sdf)
from scenes.geometry import read_pgm


def profile(bundle, speed=0.15, duration=15.0, pattern='cross'):
    if (type(speed) not in (int, float) or not math.isfinite(speed) or not 0.05 <= speed <= 0.3
            or type(duration) not in (int, float) or not math.isfinite(duration) or not 2 <= duration <= 30
            or pattern not in ('cross', 'shuttle')):
        raise ObstacleError('INVALID_MOTION_PROFILE')
    site = site_profile(bundle, 'temporary')
    low, high = site['corridor_y']
    return {'version': 'velocity-system-v1', 'pattern': pattern, 'speed': speed,
            'duration_sim_s': duration, 'wall_limit_s': 60.0, 'heartbeat_timeout_s': 0.5,
            'x_band': site['x_band'], 'y_min': low + 0.5, 'y_max': high - 0.5,
            'size': [0.4, 0.4, 1.2], 'acceleration_m_s2': 0.4,
            'stop_clearance_m': 0.35, 'robot_radius_m': bundle['source']['robot_radius_m']}


def select_trajectory(bundle, path, robot, config):
    if (not path or any(len(p) != 2 or any(not math.isfinite(v) for v in p) for p in path)
            or len(robot) != 2 or any(not math.isfinite(v) for v in robot)):
        raise ObstacleError('INVALID_MOTION_PATH_OR_ROBOT')
    grid = bundle['source']['map']; _, _, pixels = read_pgm(bundle['paths']['map_image'].read_bytes())
    y0, y1 = config['y_min'], config['y_max']
    for x in sorted({round(p[0] / grid['resolution']) * grid['resolution'] for p in path}):
        if not config['x_band'][0] <= x <= config['x_band'][1]:
            continue
        sweep = [x - 0.2, y0 - 0.2, x + 0.2, y1 + 0.2]
        if not any(point_box_distance(*p, sweep) <= 0.05 for p in path):
            continue
        # Inspect every touched map cell, including partial cells, with margin.
        ox, oy, _ = grid['origin']; res = grid['resolution']
        a, b = math.floor((sweep[0] - ox) / res), math.floor((sweep[1] - oy) / res)
        c, d = math.ceil((sweep[2] - ox) / res), math.ceil((sweep[3] - oy) / res)
        if not 0 <= a < c <= grid['width'] or not 0 <= b < d <= grid['height']:
            continue
        if any(pixels[(grid['height'] - 1 - j) * grid['width'] + i] != 255
               for j in range(b, d) for i in range(a, c)):
            continue
        # Cover the entire route, not just the spawn footprint.
        required = config['robot_radius_m'] + 1.0 + (0.5 + config['speed']) * 2.5
        if point_box_distance(*robot, sweep) < required:
            continue
        position = [round(x, 6), y0, 0.6]
        return {'position': position, 'size': config['size'], 'bounds': bounds_for(position, config['size']),
                'swept_bounds': sweep, 'required_robot_separation': required}
    raise ObstacleError('NO_SAFE_ACTUAL_PATH_CROSSING')


def motion_sdf(record):
    """Dynamic rigid body, never a sequence of set_pose requests."""
    validate_motion_record(record)
    root = ET.fromstring(obstacle_sdf(record['entity_name'], record['selection'], record['lease_id']))
    model = root.find('model'); model.find('static').text = 'false'
    link = model.find('link'); ET.SubElement(link, 'gravity').text = 'false'
    inertial = ET.SubElement(link, 'inertial'); ET.SubElement(inertial, 'mass').text = '10'
    inertia = ET.SubElement(inertial, 'inertia')
    for tag, number in [('ixx', 1.3333333333), ('iyy', 1.3333333333), ('izz', 0.2666666667),
                        ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        ET.SubElement(inertia, tag).text = str(number)
    sensor = ET.SubElement(link, 'sensor', name='ea_contact_' + record['lease_id'], type='contact')
    ET.SubElement(ET.SubElement(sensor, 'contact'), 'collision').text = 'ea_collision_' + record['lease_id']
    plugin = ET.SubElement(model, 'plugin', filename=record['motion']['plugin_path'],
                           name=record['motion'].get('plugin_class','embodied::BoundedMotion'))
    for key, val in plugin_parameters(record).items():
        ET.SubElement(plugin, key).text = str(val)
    return ET.tostring(root, encoding='unicode')


def plugin_parameters(record):
    config = record['motion']
    return {'token': record['lease_id'], 'x': record['selection']['position'][0],
            'y_min': config['y_min'], 'y_max': config['y_max'], 'speed': config['speed'],
            'duration': config['duration_sim_s'], 'pattern': config['pattern']}


def validate_motion_record(record):
    config = record['motion']; selection = record['selection']
    if (config.get('version') != 'velocity-system-v1' or config.get('pattern') not in ('cross', 'shuttle')
            or config.get('plugin_class','embodied::BoundedMotion') not in
                ('embodied::BoundedMotion','embodied::BoundedMotionHandshakeV1','embodied::BoundedMotionHeartbeatV2')
            or config.get('y_min') != 25.5 or config.get('y_max') != 26.5
            or not 15.5 <= selection['position'][0] <= 17.3
            or selection['position'][1:] != [25.5, 0.6] or selection['size'] != [0.4, 0.4, 1.2]
            or not Path(config.get('plugin_path', '')).is_absolute()
            or not math.isfinite(config['speed']) or not 0.05 <= config['speed'] <= 0.3
            or not math.isfinite(config['duration_sim_s']) or not 2 <= config['duration_sim_s'] <= 30):
        raise ObstacleError('INVALID_MOTION_OWNERSHIP_PROFILE')


def verify_motion_pose(model, record):
    validate_motion_record(record)
    p = list(map(float, model.findtext('pose', '').split()))
    x = record['selection']['position'][0]; config = record['motion']
    if (len(p) != 6 or any(not math.isfinite(v) for v in p)
            or abs(p[0] - x) > 0.04 or not config['y_min'] - 0.04 <= p[1] <= config['y_max'] + 0.04
            or abs(p[2] - 0.6) > 0.04 or any(abs(v) > 0.02 for v in p[3:])):
        raise ObstacleError('MOTION_POSE_OUTSIDE_OWNED_ROUTE')
    plugins = model.findall('plugin')
    if (len(plugins) != 1 or plugins[0].get('filename') != config['plugin_path']
            or plugins[0].get('name') != config.get('plugin_class','embodied::BoundedMotion')):
        raise ObstacleError('MOTION_PLUGIN_MISMATCH')
    for key, val in plugin_parameters(record).items():
        actual = plugins[0].findtext(key)
        if isinstance(val, (int, float)):
            if actual is None or not math.isfinite(float(actual)) or abs(float(actual) - val) > 1e-8:
                raise ObstacleError('MOTION_PLUGIN_CONFIG_MISMATCH')
        elif actual != val:
            raise ObstacleError('MOTION_PLUGIN_CONFIG_MISMATCH')
