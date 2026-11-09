"""Exact grid-aligned box geometry and independently parsed SDF projections."""
import json
import math
import xml.etree.ElementTree as ET
from collections import deque
from decimal import Decimal


def dec(value):
    number = Decimal(str(value))
    if not number.is_finite():
        raise ValueError('Non-finite geometry')
    return number


def grid_rect(bounds, grid):
    if len(bounds) != 4:
        raise ValueError('Invalid rectangle')
    x0,y0,x1,y1 = map(dec,bounds)
    if not x0 < x1 or not y0 < y1:
        raise ValueError('Empty rectangle')
    origin = grid['origin']; res = dec(grid['resolution'])
    values = [(x0-dec(origin[0]))/res,(y0-dec(origin[1]))/res,
              (x1-dec(origin[0]))/res,(y1-dec(origin[1]))/res]
    if any(v != int(v) for v in values):
        raise ValueError('Geometry must align exactly to map cell edges')
    a,b,c,d = map(int,values)
    if not 0 <= a < c <= grid['width'] or not 0 <= b < d <= grid['height']:
        raise ValueError('Geometry lies outside map')
    return a,b,c,d


def raster(rectangles, grid):
    width,height=grid['width'],grid['height']
    # PGM row 0 is north; ROS occupancy row 0 is south.
    pixels=bytearray([255])*(width*height)
    for bounds in rectangles:
        x0,y0,x1,y1=grid_rect(bounds,grid)
        for south_row in range(y0,y1):
            start=(height-1-south_row)*width+x0
            pixels[start:start+x1-x0]=b'\x00'*(x1-x0)
    return bytes(pixels)


def pgm_bytes(pixels, width, height):
    return f'P5\n{width} {height}\n255\n'.encode()+pixels


def read_pgm(data):
    tokens=[];position=0
    while len(tokens)<4:
        while data[position:position+1] in (b' ',b'\t',b'\r',b'\n'):
            position+=1
        if data[position:position+1]==b'#':
            end=data.find(b'\n',position)
            if end<0:raise ValueError('Truncated PGM')
            position=end+1;continue
        start=position
        while position<len(data) and data[position:position+1] not in (b' ',b'\t',b'\r',b'\n'):
            position+=1
        if position==start:raise ValueError('Truncated PGM')
        tokens.append(data[start:position])
    if tokens[0]!=b'P5' or tokens[3]!=b'255':raise ValueError('Only 8-bit P5 maps supported')
    width,height=int(tokens[1]),int(tokens[2])
    position+=2 if data[position:position+2]==b'\r\n' else 1
    pixels=data[position:]
    if width<=0 or height<=0 or len(pixels)!=width*height:raise ValueError('Invalid PGM dimensions')
    return width,height,pixels


def _pose(element):
    values=list(map(dec,element.findtext('pose','0 0 0 0 0 0').split()))
    if len(values)!=6 or any(v!=0 for v in values[3:]):
        raise ValueError('Only unrotated box profile supported')
    return values[:3]


def rectangles_from_sdf(data):
    root=ET.fromstring(data);world=root.find('world')
    if world is None or world.get('name')!='default' or world.findall('.//include'):
        raise ValueError('Unexpected world or external model dependency')
    rectangles=[]
    for model in world.findall('model'):
        if model.findtext('static') not in ('true','1'):raise ValueError('Non-static geometry')
        mp=_pose(model)
        for link in model.findall('link'):
            lp=_pose(link)
            for collision in link.findall('collision'):
                cp=_pose(collision)
                size=collision.findtext('geometry/box/size')
                if size is None:raise ValueError('Unverified collision type')
                sizes=list(map(dec,size.split()))
                if len(sizes)!=3 or any(v<=0 for v in sizes):raise ValueError('Invalid collision box')
                x,y,z=[mp[i]+lp[i]+cp[i] for i in range(3)]
                if z+sizes[2]/2<=0:continue  # Ground floor does not intersect laser/body plane.
                if z-sizes[2]/2!=0 or z+sizes[2]/2<dec('0.5'):
                    raise ValueError('Floating/short obstacle outside validated scan profile')
                rectangles.append([x-sizes[0]/2,y-sizes[1]/2,x+sizes[0]/2,y+sizes[1]/2])
    return rectangles


def static_navigation_check(pixels, grid, spawn, waypoints, clearance):
    width,height=grid['width'],grid['height'];res=grid['resolution']
    occupied=[(i%width,height-1-i//width) for i,v in enumerate(pixels) if v==0]
    safe=bytearray([1])*(width*height)
    radius=math.ceil(clearance/res+1)
    # Inflate occupied CELL SQUARES, plus a half-cell diagonal for the candidate
    # cell. Thus the entire reachable cell has the declared disk clearance.
    offsets=[(dx,dy) for dy in range(-radius,radius+1) for dx in range(-radius,radius+1)
             if math.hypot(max(abs(dx)-0.5,0)*res,max(abs(dy)-0.5,0)*res)
                <= clearance+res/math.sqrt(2)]
    for x,y in occupied:
        for dx,dy in offsets:
            nx,ny=x+dx,y+dy
            if 0<=nx<width and 0<=ny<height:safe[ny*width+nx]=0
    # Map edge is a hard boundary for the footprint, too.
    margin=math.ceil(clearance/res+0.5)
    for y in range(height):
        for x in range(width):
            if min(x,y,width-1-x,height-1-y)<margin:safe[y*width+x]=0
    def cell(point):
        x=math.floor((point['x']-grid['origin'][0])/res)
        y=math.floor((point['y']-grid['origin'][1])/res)
        if not 0<=x<width or not 0<=y<height or not safe[y*width+x]:
            raise ValueError('Spawn/waypoint has insufficient static clearance')
        return y*width+x
    start=cell(spawn);queue=deque([start]);seen={start}
    while queue:
        i=queue.popleft();x,y=i%width,i//width
        for nx,ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            j=ny*width+nx
            if 0<=nx<width and 0<=ny<height and safe[j] and j not in seen:
                seen.add(j);queue.append(j)
    if any(cell(point) not in seen for point in waypoints.values()):
        raise ValueError('Waypoints not connected to spawn at required clearance')
    return {'reachable_safe_cells':len(seen),'clearance_m':clearance,
            'occupied_cells':len(occupied),'waypoint_count':len(waypoints)}
