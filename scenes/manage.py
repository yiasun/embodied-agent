"""Offline scene selection. Prints verified launch arguments; never starts ROS."""
import argparse
import hashlib
import json
from pathlib import Path

from planner.planner_v1 import load_scene
from .assets import validate_scene_assets

ROOT=Path(__file__).resolve().parents[1]


def launch_arguments(scene_id,*,root=ROOT):
    # load_scene validates the registry and managed asset pins itself.
    scene=load_scene(scene_id)
    bundle=validate_scene_assets(scene,root)
    if bundle:
        paths=bundle['paths'];spawn=bundle['source']['spawn']
        return {'world':str(paths['world']),'map':str(paths['map']),
                'params_file':str(paths['nav2_params']),
                'x_pose':str(spawn['x']),'y_pose':str(spawn['y']),'z_pose':str(spawn['z']),
                'yaw':str(spawn['yaw']),'use_sim_time':'true','slam':'False',
                'use_composition':'False','autostart':'true'}
    index=json.loads((root/'configs/scene_resources/index.json').read_text())
    binding=index['legacy_scenes'].get(scene_id)
    if not binding:raise ValueError('Scene has no simulator resource binding')
    paths={}
    for kind,record in binding['files'].items():
        path=Path(record['path'])
        if hashlib.sha256(path.read_bytes()).hexdigest()!=record['sha256']:
            raise ValueError('Legacy system resource changed; re-audit required')
        paths[kind]=path
    return {'world':str(paths['world']),'map':str(paths['map']),'params_file':str(paths['nav2_params']),
            'x_pose':'-2.0','y_pose':'-0.5','z_pose':'0.01','yaw':'0.0',
            'use_sim_time':'true','slam':'False','use_composition':'False','autostart':'true'}


def main(argv=None):
    parser=argparse.ArgumentParser(description='Only verifies assets/prints arguments; never launches simulation')
    parser.add_argument('--scene',default='inspection_world')
    args=parser.parse_args(argv)
    try:
        scene=load_scene(args.scene);bundle=validate_scene_assets(scene,ROOT)
        output={'scene_id':args.scene,'launch_arguments':launch_arguments(args.scene),
                'verification':'offline-only','runtime_navigation_verified':False}
        if bundle:output.update(asset_identity=bundle['identity'],static_map_checks=bundle['metrics'])
        print(json.dumps(output,ensure_ascii=False,indent=2))
    except (ValueError,OSError,KeyError,TypeError):
        print('{"error":"SCENE_RESOURCE_CHECK_FAILED"}')
        return 1
    return 0


if __name__=='__main__':
    raise SystemExit(main())
