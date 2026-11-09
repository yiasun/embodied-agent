# 第二阶段安全修复交付

本阶段只运行离线测试，没有启动Gazebo/RViz、发送Nav2目标、调用模型API或安装/升级ROS依赖。

## 基线保护

`.git`被只读空tmpfs挂载遮蔽，底层Git数据不可见；没有修改挂载或初始化Git。
修改前26个源码/配置/计划/实验文件已快照至`.snapshots/safety_20261108T072741Z/`。
2份计划、8份实验日志及1份场景注册表逐文件SHA-256保持不变。

## 改动范围

修改：

- `planner/planner_v1.py`：统一1—20任务，校验注册表、场景ID和路径边界。
- `planner/nl_parser_v1.py`：兼容包导入；规则句法保持不变。
- `planner/execute_plan.py`：严格校验、默认预检查、计划摘要、报告交叉核验、中断转发。
- `day4/task_executor_v2.py`：保留ROS Node、map目标、零度朝向和距离反馈，接入隔离状态机；旧CLI转交V4。
- `day4/task_executor_v4.py`：保留顺序执行及Recovery Policy，支持单目标；补非零失败码、原子报告、初始化/清理异常、中断处理和显式执行开关。

新增：

- `planner/__init__.py`、`day4/__init__.py`。
- `planner/validation.py`、`planner/schemas/navigation_plan.schema.json`。
- `day4/navigation_session.py`、`day4/reporting.py`。
- `tests/conftest.py`、`tests/test_plans.py`、`tests/test_navigation.py`、`tests/test_executor.py`、`tests/test_ros_adapter.py`。
- `pyproject.toml`、`.gitignore`、`README.md`、本交付文档。

`day4/recovery_policy.py`原文件保持不变。历史V1/V3和day3实验脚本保留，未重构为当前安全入口；它们不是默认测试命令。

## 验证

最终离线回归：155项，155通过，0失败。

覆盖合法单/双/20目标计划、非法Schema/动作/地点/坐标、路径穿越与越界符号链接、重复JSON键、全部退出状态、取消终态、发送超时、迟到响应、重试停止、中断/SystemExit、初始化和日志写入/替换/清理失败、上层报告核验、脚本及模块入口、ROS适配器模拟契约。

测试网络被禁止，ROS适配器使用模块替身，所有导航器均为模拟对象。

```bash
cd ~/embodied-agent
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider
python3 -B -m planner.execute_plan --plan plans/plan_nl_v1.json
```

## 接口变化

原`execute_plan --execute`保留，缺省仍为预检查。
V4及旧V2坐标CLI现在也必须显式提供`--execute`，否则只预检查；坐标/timeout/retries及ROS尾部参数保留。
V4增加`--report`、`--execution-id`、`--plan-id`；不允许覆盖现有报告。
V2旧CLI与V4共享退出码及任务编排，但`RobotNavigator`类仍供V3复用。
失败退出码和报告验证属于有意修复，不应继续依赖历史的失败零退出码。

## 限制与待验收项

- 修改后的真实ROS2/Nav2行为尚未回归，离线模拟不能替代仿真验收。
- UNKNOWN只说明无法确认目标终止；不会自动执行后续任务，也不代表底盘已停止。重新运行前应确认Nav2实际状态。
- 无法获取目标句柄时，不能精确取消该目标；不发送全局取消以免影响其他客户端。
- SIGKILL、宿主/中间件崩溃及永久磁盘故障不保证取消或最终报告写入。
- 底层数字坐标工具不是地点语义校验入口，Agent必须使用注册表与计划校验链路。
- 历史实验入口仍保留其主动导航行为；不要用历史取消实验脚本替代离线pytest。
- Git提交与真实工作区差异需在未遮蔽Git信息的WSL会话中核查；本阶段未创建提交。

Qwen/Ollama/真实API测试不属于本阶段，均未实现或执行。
