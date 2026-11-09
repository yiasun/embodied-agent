"""Exercise the ROS adapter contract using module stubs, never real ROS."""
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

from day4 import task_executor_v4


def test_ros_goal_adapter_contract(monkeypatch):
    rclpy=ModuleType('rclpy');rclpy.spin_until_future_complete=Mock()
    node_module=ModuleType('rclpy.node')
    class Node:
        def __init__(self,name):self.name=name
        def get_clock(self):return SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:'stamp'))
        def get_logger(self):return SimpleNamespace(info=Mock())
    node_module.Node=Node
    action_module=ModuleType('rclpy.action');action_module.ActionClient=Mock()
    nav_module=ModuleType('nav2_msgs.action')
    class Goal:
        def __init__(self):
            self.pose=SimpleNamespace(header=SimpleNamespace(frame_id=None,stamp=None),
                pose=SimpleNamespace(position=SimpleNamespace(x=None,y=None),orientation=SimpleNamespace(w=None)))
    nav_module.NavigateToPose=SimpleNamespace(Goal=Goal)
    for name,module in [('rclpy',rclpy),('rclpy.node',node_module),('rclpy.action',action_module),
                        ('nav2_msgs',ModuleType('nav2_msgs')),('nav2_msgs.action',nav_module)]:
        monkeypatch.setitem(sys.modules,name,module)
    path=Path(__file__).resolve().parents[1]/'day4/task_executor_v2.py'
    spec=importlib.util.spec_from_file_location('day4.adapter_under_test',path)
    adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)
    node=adapter.RobotNavigator()
    action_module.ActionClient.assert_called_once_with(node,nav_module.NavigateToPose,'/navigate_to_pose')
    goal=node._make_goal(0.419,-1.822)
    assert goal.pose.header.frame_id=='map' and goal.pose.header.stamp=='stamp'
    assert (goal.pose.pose.position.x,goal.pose.pose.position.y)==(0.419,-1.822)
    assert goal.pose.pose.orientation.w==1.0
    node.session.navigate=Mock(return_value=False)
    node.session.last_status='UNKNOWN';node.session.last_reason='send_response_timeout'
    assert not node.navigate(0,0)
    assert node.last_status=='UNKNOWN'
    node.session.abort_active=Mock(return_value=False)
    assert not node.abort_active()


def test_explicit_cli_ros_lifecycle_is_mocked(tmp_path,monkeypatch):
    # No robot calls: both the imported module and navigator are fakes.
    rclpy=ModuleType('rclpy');rclpy.init=Mock();rclpy.ok=Mock(return_value=True);rclpy.shutdown=Mock()
    adapter=ModuleType('day4.task_executor_v2')
    node=SimpleNamespace(navigate=Mock(return_value=True),last_status='SUCCEEDED',last_reason='goal_reached',
                         abort_active=Mock(return_value=True),destroy_node=Mock())
    adapter.RobotNavigator=Mock(return_value=node)
    monkeypatch.setitem(sys.modules,'rclpy',rclpy)
    monkeypatch.setitem(sys.modules,'day4.task_executor_v2',adapter)
    import signal
    handler=Mock(return_value='old-handler')
    monkeypatch.setattr(task_executor_v4.signal,'signal',handler)
    code=task_executor_v4.main(['0','0','--execute','--report',str(tmp_path/'report.json'),
        '--ros-args','-p','use_sim_time:=true'])
    assert code==0
    rclpy.init.assert_called_once();rclpy.shutdown.assert_called_once()
    assert rclpy.init.call_args.kwargs['args'][1:]==['--ros-args','-p','use_sim_time:=true']
    callback=handler.call_args_list[0].args[1]
    import pytest
    with pytest.raises(KeyboardInterrupt):callback(signal.SIGTERM,None)
    assert handler.call_args_list[-1].args==(signal.SIGTERM,'old-handler')
