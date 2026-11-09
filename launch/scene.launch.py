"""Explicit user-operated scene launch, with pinned world/map/params selection.

This file is not imported by offline planning. No arbitrary world/map override is
exposed: resources are chosen together from the validated scene registry.
"""
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def _setup(context):
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration
    from ament_index_python.packages import get_package_share_directory
    from scenes.manage import launch_arguments
    selected=LaunchConfiguration('scene_id').perform(context)
    arguments=launch_arguments(selected)
    arguments['headless']=LaunchConfiguration('headless').perform(context)
    arguments['use_rviz']=LaunchConfiguration('use_rviz').perform(context)
    target=Path(get_package_share_directory('nav2_bringup'))/'launch/tb3_simulation_launch.py'
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(target)),launch_arguments=arguments.items())]


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument,OpaqueFunction
    return LaunchDescription([
        DeclareLaunchArgument('scene_id',default_value='inspection_world'),
        DeclareLaunchArgument('headless',default_value='False'),
        DeclareLaunchArgument('use_rviz',default_value='True'),
        OpaqueFunction(function=_setup),
    ])
