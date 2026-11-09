"""Default: read-only reproducibility check. --write is explicit authoring only."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scenes.build_assets import build,generated_assets


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write',action='store_true',help='Regenerate unpublished v1 files; never invoked by planning')
    args=parser.parse_args(argv)
    root=Path(__file__).resolve().parents[1]
    if args.write:
        manifest=build(root)
        print('Generated authored geometry assets:',manifest['scene_id'],manifest['revision'])
        return 0
    directory=root/'assets/scenes/inspection_world/v1'
    source=json.loads((directory/'geometry.json').read_text())
    template=json.loads((directory/'nav2_template.json').read_text())
    names={'world':'inspection_world.sdf','map':'inspection_world.yaml',
           'map_image':'inspection_world.pgm','nav2_params':'nav2_params.yaml'}
    for kind,data in generated_assets(source,template).items():
        if (directory/names[kind]).read_bytes()!=data:
            print('Mismatch; no files changed:',kind)
            return 1
    print('Reproducible world/map/parameters; no files changed')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
