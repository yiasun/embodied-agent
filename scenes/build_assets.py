"""Deterministic source -> SDF, exact occupancy map, local Nav2 parameters."""
import copy
import hashlib
import json
import xml.etree.ElementTree as ET
from decimal import Decimal
from pathlib import Path

from .geometry import dec, raster, pgm_bytes


def formatted(number):
    return format(number,'f') if isinstance(number,Decimal) else str(number)


def _text(parent,tag,value):
    node=ET.SubElement(parent,tag);node.text=value;return node


def _box(world,name,bounds,height,color,collision=True,z=None):
    x0,y0,x1,y1=map(dec,bounds);height=dec(height)
    model=ET.SubElement(world,'model',name=name)
    _text(model,'static','true')
    _text(model,'pose',' '.join(map(formatted,[(x0+x1)/2,(y0+y1)/2,height/2 if z is None else dec(z),0,0,0])))
    link=ET.SubElement(model,'link',name='body')
    size=' '.join(map(formatted,[x1-x0,y1-y0,height]))
    if collision:
        node=ET.SubElement(link,'collision',name='collision')
        _text(ET.SubElement(ET.SubElement(node,'geometry'),'box'),'size',size)
    visual=ET.SubElement(link,'visual',name='visual')
    _text(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size',size)
    material=ET.SubElement(visual,'material');_text(material,'ambient',color);_text(material,'diffuse',color)


def world_bytes(source):
    root=ET.Element('sdf',version='1.9');world=ET.SubElement(root,'world',name='default')
    for filename,name in [('physics','Physics'),('user-commands','UserCommands'),
                          ('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),('imu','Imu')]:
        plugin=ET.SubElement(world,'plugin',filename=f'gz-sim-{filename}-system',name=f'gz::sim::systems::{name}')
        if filename=='sensors':_text(plugin,'render_engine','ogre2')
    physics=ET.SubElement(world,'physics',name='3ms',type='ode')
    _text(physics,'max_step_size','0.003');_text(physics,'real_time_factor','1')
    light=ET.SubElement(world,'light',name='sun',type='directional')
    _text(light,'pose','18 26 10 0 0 0');_text(light,'diffuse','0.8 0.8 0.8 1');_text(light,'direction','-0.5 0.1 -0.9')
    grid=source['map'];ox,oy,_=grid['origin']
    bounds=[ox,oy,ox+grid['width']*grid['resolution'],oy+grid['height']*grid['resolution']]
    _box(world,'floor',bounds,'0.1','0.7 0.7 0.7 1',z='-0.05')
    for region in source['regions']:
        _box(world,'region_'+region['name'],region['bounds'],'0.002',region['color'],collision=False,z='0.001')
    for box in source['boxes']:
        _box(world,box['name'],box['bounds'],box.get('height',source['wall_height']),'0.65 0.65 0.65 1')
    ET.indent(root,space='  ')
    return ET.tostring(root,encoding='utf-8',xml_declaration=True)+b'\n'


def json_bytes(value):
    return (json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode('utf-8')


def generated_assets(source,template):
    grid=source['map']
    pixels=raster([b['bounds'] for b in source['boxes']],grid)
    map_yaml={'image':source['scene_id']+'.pgm','mode':'trinary','resolution':grid['resolution'],
              'origin':grid['origin'],'negate':0,'occupied_thresh':0.65,'free_thresh':0.196}
    params=copy.deepcopy(template)
    amcl=params['amcl']['ros__parameters']
    amcl['set_initial_pose']=True
    amcl['initial_pose']={key:source['spawn'].get(key,0.0) for key in ('x','y','z','yaw')}
    amcl['initial_pose']['z']=0.0
    return {'world':world_bytes(source), 'map':json_bytes(map_yaml),
            'map_image':pgm_bytes(pixels,grid['width'],grid['height']), 'nav2_params':json_bytes(params)}


def build(root):
    root=Path(root);directory=root/'assets/scenes/inspection_world/v1'
    source=json.loads((directory/'geometry.json').read_text())
    template=json.loads((directory/'nav2_template.json').read_text())
    output=generated_assets(source,template)
    names={'geometry':'geometry.json','nav2_template':'nav2_template.json','world':'inspection_world.sdf',
           'map':'inspection_world.yaml','map_image':'inspection_world.pgm','nav2_params':'nav2_params.yaml'}
    # This is an explicit developer regeneration command, never called by planning.
    for kind,data in output.items():(directory/names[kind]).write_bytes(data)
    manifest={'asset_schema_version':'1.0','scene_id':source['scene_id'],'revision':source['revision'],
              'profile':source['geometry_version'],'map_to_world':[0.0,0.0,0.0],
              'verification_level':'offline-geometry-and-connectivity-only',
              'files':{kind:{'path':str((directory/name).relative_to(root)),
                             'sha256':hashlib.sha256((directory/name).read_bytes()).hexdigest()}
                       for kind,name in names.items()}}
    (directory/'manifest.json').write_bytes(json_bytes(manifest))
    config={'scene_id':source['scene_id'],'description':'独立四室巡检场景；离线几何已校验，真实仿真待验收',
            'frame_id':'map','waypoints':source['waypoints'],
            'assets':{'revision':source['revision'],'manifest':str((directory/'manifest.json').relative_to(root)),
                      'manifest_sha256':hashlib.sha256((directory/'manifest.json').read_bytes()).hexdigest()}}
    (root/'configs/scenes/inspection_world.json').write_bytes(json_bytes(config))
    return manifest
