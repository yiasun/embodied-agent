"""Fail-closed managed scene validation. No simulation, network or ROS imports."""
import copy
import hashlib
import json
import re
from functools import lru_cache
from pathlib import Path

from .build_assets import generated_assets
from .geometry import read_pgm, rectangles_from_sdf, raster, static_navigation_check


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def _read_json(data):
    def pairs(items):
        out={}
        for key,value in items:
            if key in out:raise ValueError('Duplicate asset JSON key')
            out[key]=value
        return out
    def reject(_):raise ValueError('Non-finite asset JSON')
    return json.loads(data,object_pairs_hook=pairs,parse_constant=reject)


def _digest(value):
    if not isinstance(value,str) or not re.fullmatch('[0-9a-f]{64}',value):raise ValueError('Invalid asset digest')


def _path(root, relative, within):
    if not isinstance(relative,str) or Path(relative).is_absolute():raise ValueError('Asset paths must be project-relative')
    path=(root/relative).resolve()
    if not path.is_relative_to(within.resolve()):raise ValueError('Scene asset path escapes scene directory')
    if not path.is_file():raise ValueError('Scene resource missing')
    return path


@lru_cache(maxsize=8)
def _check_bundle(geometry,template,world,map_yaml,map_image,nav2_params):
    source=_read_json(geometry);base=_read_json(template)
    if source.get('geometry_version')!='axis-aligned-indoor-v1' or source.get('world_name')!='default':
        raise ValueError('Unsupported offline geometry profile')
    grid=source['map']
    if (type(grid['width']) is not int or type(grid['height']) is not int
            or not 0<grid['width']<=4096 or not 0<grid['height']<=4096
            or grid['width']*grid['height']>1000000
            or not 0<float(grid['resolution'])<=1 or len(grid['origin'])!=3 or grid['origin'][2]!=0):
        raise ValueError('Invalid map extent')
    if not 0<float(source['robot_radius_m'])<=float(source['clearance_m'])<=2:
        raise ValueError('Invalid footprint clearance')
    if not 1<=len(source['boxes'])<=200 or len({b['name'] for b in source['boxes']})!=len(source['boxes']):
        raise ValueError('Invalid collision inventory')
    expected=generated_assets(source,base)
    actual={'world':world,'map':map_yaml,'map_image':map_image,'nav2_params':nav2_params}
    if any(actual[k]!=expected[k] for k in actual):
        raise ValueError('World/map/navigation assets differ from authored geometry')
    config=_read_json(map_yaml)
    if config['image']!=source['scene_id']+'.pgm' or config['origin']!=grid['origin'] or config['resolution']!=grid['resolution']:
        raise ValueError('Map coordinate configuration mismatch')
    width,height,pixels=read_pgm(map_image)
    if (width,height)!=(grid['width'],grid['height']):raise ValueError('Map dimensions mismatch')
    # Independent SDF collision parser, not just a hash/name comparison.
    projected=raster(rectangles_from_sdf(world),grid)
    if projected!=pixels:raise ValueError('Occupancy raster does not match SDF collision geometry')
    params=_read_json(nav2_params)
    for key in ('local_costmap','global_costmap'):
        settings=params[key][key]['ros__parameters']
        if settings['robot_radius']!=source['robot_radius_m'] or settings['resolution']!=grid['resolution']:
            raise ValueError('Navigation footprint or grid resolution mismatch')
    metrics=static_navigation_check(pixels,grid,source['spawn'],source['waypoints'],source['clearance_m'])
    return source,metrics


def validate_scene_assets(scene,root):
    root=Path(root).resolve();scene_id=scene['scene_id']
    index_path=root/'configs/scene_resources/index.json'
    index=_read_json(index_path.read_bytes()) if index_path.is_file() else {'managed_scenes':{}}
    if (index_path.is_file() and (index.get('resource_index_version')!='1.0'
            or not isinstance(index.get('managed_scenes'),dict))):
        raise ValueError('Invalid scene resource index')
    pin=index.get('managed_scenes',{}).get(scene_id)
    metadata=scene.get('assets')
    if pin is not None and metadata!=pin:raise ValueError('Managed scene asset/version pin missing or changed')
    if metadata is None:return None  # Legacy schemas retain their original identity.
    if not isinstance(metadata,dict) or set(metadata)!={'revision','manifest','manifest_sha256'}:
        raise ValueError('Invalid scene asset binding')
    if not isinstance(metadata['revision'],str) or not re.fullmatch(r'[0-9]+\.[0-9]+\.[0-9]+',metadata['revision']):
        raise ValueError('Invalid scene asset revision')
    _digest(metadata['manifest_sha256'])
    directory=root/'assets/scenes'/scene_id
    manifest_path=_path(root,metadata['manifest'],directory)
    raw=manifest_path.read_bytes()
    if sha256(raw)!=metadata['manifest_sha256']:raise ValueError('Scene manifest hash mismatch')
    manifest=_read_json(raw)
    if (manifest.get('asset_schema_version')!='1.0' or manifest.get('scene_id')!=scene_id
            or manifest.get('revision')!=metadata['revision'] or manifest.get('map_to_world')!=[0,0,0]
            or manifest.get('profile')!='axis-aligned-indoor-v1'):
        raise ValueError('Scene manifest identity or coordinates mismatch')
    files=manifest.get('files',{})
    roles={'geometry','nav2_template','world','map','map_image','nav2_params'}
    if set(files)!=roles:raise ValueError('Incomplete scene resources')
    data={};paths={}
    for role,record in files.items():
        _digest(record['sha256'])
        path=_path(root,record['path'],manifest_path.parent)
        raw=path.read_bytes()
        if sha256(raw)!=record['sha256']:raise ValueError('Scene resource hash mismatch: '+role)
        data[role]=raw;paths[role]=path
    map_config=_read_json(data['map'])
    if (paths['map'].parent/map_config['image']).resolve()!=paths['map_image']:
        raise ValueError('Map YAML image does not reference the pinned image resource')
    source,metrics=_check_bundle(*(data[k] for k in ('geometry','nav2_template','world','map','map_image','nav2_params')))
    if (source['scene_id']!=scene_id or source['revision']!=metadata['revision']
            or source['waypoints']!=scene['waypoints']):
        raise ValueError('Scene registry differs from versioned geometry')
    return {'identity':{'revision':metadata['revision'],'manifest_sha256':metadata['manifest_sha256']},
            'paths':paths,'source':copy.deepcopy(source),'metrics':copy.deepcopy(metrics)}
