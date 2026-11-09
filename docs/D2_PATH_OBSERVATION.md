# D2 路径观测修复

原调用链是moving.main → run_lease → Gazebo发现/世界检查 → active_task与target-step检查 → 构造EvaluationObserver（父类NavigationObserver在此时订阅/map和/plan）→ sample。VOLATILE不会重放订阅建立前的路径，因而在第3站开始后才启动工具可能错过该站规划；同时只保留最新Path会被另一个发布者的无关路径覆盖。原5秒等待超时不能区分具体原因。

修复前源码快照与逐文件SHA256在`.snapshots/d2_path_fix_20261109T044727Z/`。本开发会话看到的.git为空的只读挂载，无可用Git历史；没有修改或重新初始化它。历史D1/D2、计划、正式资产保持原样。

## 新的准备流程

仅对moving的D2/D3 Apply启用：

1. 保留只读预检、唯一运行绑定、进程锁和新文件保留规则。
2. 在Gazebo发现和世界检查前建立订阅，并用独立SingleThreadedExecutor持续处理只读回调。主线程等待和GazeboRPC不会阻止记录真实接收时刻。
3. 用`--wait-task-seconds`（默认120墙钟秒，范围0…300）等待指定任务RUNNING，期间重复验证Plan/Run/Provenance。0恢复“必须已经RUNNING”的要求。只等待已有导航，不启动/重放导航，不请求额外规划。
4. 进入目标任务后继续执行原世界/地图/轨迹/机器人距离校验，并从最多64个候选中选取新鲜、frame与目标合法的路径。两个发布者共享话题时按消息内容逐项验证，不依赖猜测节点名、不直接信任任意发布者。
5. 创建前再次验证已通过几何门禁的**同一候选**和RUNNING任务。采样返回后过期、任务结束或身份变化均拒绝创建。
6. 原生模型确认创建后，先停止并join准备线程、解除专用executor，再交给原EvaluationObserver.poll前台采集，禁止两个executor并发操作同一节点。异常仍走原清理/UNKNOWN协议。

未改变这些条件：/map必须匹配正式资产；Path frame=map；2…10000个有限坐标点；接收墙钟年龄≤2秒；Path时间戳>0且相对ROS仿真时间差处于[-0.25,3]秒；终点距离目标≤0.6m；路径起点/通道/整个扫掠区/机器人净距和会话校验仍然有效。

不会增加旧路径TTL、改成TRANSIENT_LOCAL、强制Nav2重规划或在已结束任务上创建障碍。若启动过晚而缺少任何新鲜匹配路径，仍应失败。新鲜度检查需要真实消息，不能仅延长sample等待来“补回”已丢失的VOLATILE消息。

## 用户后续复测命令（本次未执行）

先由用户在导航终端明确运行既有稳定入口。在第1或第2站RUNNING期间，在另一个已source ROS的终端尽早运行：

```bash
cd ~/embodied-agent
source /opt/ros/jazzy/setup.bash
PLAN="$PWD/plans/inspection_qwen_probe_01/plan.json"
RUN_REPORT=$(python3 -B -m reliability.obstacles find-run --plan "$PLAN") || exit 1
TAG=$(date +%Y%m%d_%H%M%S_%N)
LEASE="$PWD/logs/reliability/D2_pathfix_${TAG}.json"
METRICS="$PWD/logs/reliability/D2_pathfix_${TAG}_metrics.json"

# 默认Dry Run：不构造节点、不等待、不创建文件或实体
python3 -B -m reliability.moving --plan "$PLAN" --run-report "$RUN_REPORT" \
  --pattern cross --speed 0.15 --duration 15 --target-step 3 \
  --wait-task-seconds 120 --journal "$LEASE" --metrics "$METRICS"

# 用户明确触发：先只读订阅/等待，然后才可能通过安全门禁注入
python3 -B -m reliability.moving --plan "$PLAN" --run-report "$RUN_REPORT" \
  --pattern cross --speed 0.15 --duration 15 --target-step 3 \
  --wait-task-seconds 120 --journal "$LEASE" --metrics "$METRICS" --apply
```

不要等到第3站路径已经发布后才开始准备。不复用已完成的RUN_REPORT或历史日志文件。路径未经过批准的通道时仍报NO_SAFE_ACTUAL_PATH_CROSSING，不通过手改坐标或放宽净距绕过。

## 失败记录

新Journal与metrics都附加`path_observation`，包括实际接收数量、缓存淘汰数、最近4个候选的终点/仿真时间戳/墙钟年龄、每次sample的选择或拒绝记录、等待状态和最新绑定任务状态。`rejection_checks`计数是校验次数，同一消息可能被多次检查；不是发布次数。

| 诊断 | 含义 |
|---|---|
| MAP_NOT_RECEIVED | 未收到/map |
| PATH_NOT_RECEIVED | 未收到/plan |
| PATH_WALL_EXPIRED | 已接收候选超过2秒墙钟年龄 |
| PATH_SIM_TIME_MISMATCH | 时间戳未初始化、非有限或与仿真时间不匹配 |
| PATH_TARGET_MISMATCH | 路径终点与当前目标距离超过0.6m |
| TASK_ENDED_EARLY / preparation状态 | 等待或采样期间导航/目标任务已结束 |
| PATH_FRAME_MISMATCH / INVALID_PATH_SIZE / NONFINITE_PATH | 路径结构或坐标非法 |
| MAP_ASSET_MISMATCH | 活跃地图不匹配正式资产 |
| PATH_OBSERVER_WORKER_FAILED | 准备线程回调失败，失败关闭 |

没有发送create时，正常失败路径仍记录create_sent=false、entity_id=null、cleanup=NOT_NEEDED。等待期间定期持久化进度；中断会释放只读订阅和原进程锁，不控制机器人。

## 边界与回归

缓存有界；若大量消息挤掉候选会记录buffer_evictions，仍按新鲜度失败关闭，不保证任意洪泛下捕获路径。实时ROS的发布、任务报告写入和create之间仍存在有限竞态，故保留多次绑定与最后检查；离线Mock不能代替实际D2复测。

D1的capture.py、evaluate.py、eval_observer.py、metrics.py及已验证pose backlog修复未修改。共享父类仍保留`path=(最新消息,接收时刻)`接口，D1不会启用后台准备或任务等待；sample使用同一组安全条件。

离线回归命令：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider
```

本次最终结果：原695项全部保持通过，新增37项，共732项通过、0失败。覆盖提前订阅/晚订阅、双发布者覆盖、缓存过期、时间/目标/frame/地图不匹配、任务结束、身份改变、最后创建门禁、线程交接及失败记录。没有执行真实Gazebo、模型API、导航或障碍注入；D2仍需用户后续实际复测。
