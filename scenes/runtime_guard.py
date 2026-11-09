"""Read-only live-map gate, invoked ONLY on explicit asset-plan execution."""
import json
import math
import time

from .geometry import read_pgm


def expected_ros_map(bundle):
    paths=bundle['paths'];config=json.loads(paths['map'].read_text())
    width,height,pixels=read_pgm(paths['map_image'].read_bytes())
    values=[]
    for y in range(height):
        row=pixels[(height-1-y)*width:(height-y)*width]
        values.extend(100 if v==0 else 0 for v in row)
    return config,width,height,values


def check_map_message(message,bundle):
    config,width,height,values=expected_ros_map(bundle)
    info=message.info;pose=info.origin
    if (message.header.frame_id!='map' or (info.width,info.height)!=(width,height)
            or not math.isclose(info.resolution,config['resolution'],rel_tol=0,abs_tol=1e-7)
            or not math.isclose(pose.position.x,config['origin'][0],rel_tol=0,abs_tol=1e-8)
            or not math.isclose(pose.position.y,config['origin'][1],rel_tol=0,abs_tol=1e-8)
            or pose.position.z!=0 or pose.orientation.x!=0 or pose.orientation.y!=0
            or pose.orientation.z!=0 or abs(pose.orientation.w)!=1
            or list(message.data)!=values):
        raise ValueError('Active /map does not match selected scene assets; no goal may be sent')
    return True


def assert_runtime_map(node,scene,root,*,reader=None):
    from .assets import validate_scene_assets
    bundle=validate_scene_assets(scene,root)
    if bundle is None:return
    if reader is not None:
        return check_map_message(reader(node),bundle)
    # This branch is not called by offline scene checks or plan previews.
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    received=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    subscription=node.create_subscription(OccupancyGrid,'/map',received.append,qos)
    deadline=time.monotonic()+10.0
    try:
        while not received and rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=min(0.2,max(0,deadline-time.monotonic())))
        if not received:raise ValueError('Timed out reading active map; no navigation goal sent')
        check_map_message(received[0],bundle)
    finally:
        node.destroy_subscription(subscription)
