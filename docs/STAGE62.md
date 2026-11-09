# Stage 6.2：连续速度障碍与量化观测

本文保留Stage 6.2首次实现的架构与操作说明，当时仅完成Mock与静态验证。后续用户已执行D1/D2/D3真实仿真，留存报告、四图复核和本地回放见[Stage 6.2实验结果](STAGE62_RESULTS.md)。D3因安全门禁提前停止，不能描述为完整25秒运动成功。以下新实验命令仍必须由操作者明确执行；工具返回0只说明观察/租约流程正常结束，不表示机器人成功避障或零碰撞。

## Stage 6.1 审计

- C租约`C_20261109_013430_262511417.json`关联run `c45f19509bfc47db849fbfe3233dbfc2`，实体125，COMPLETED/CLEAN；报告第1、2站成功，第3站FAILED/nav2_status_6，HALT/non_retryable_failure，第4站SKIPPED。v2计划和Provenance关联一致。
- 两份B租约和对应V4报告为COMPLETED/CLEAN及四点SUCCEEDED。绕行、重新规划与后续probe无残留是用户提供的现场验收观察；历史JSON本身不能证明无接触或重规划次数。
- 历史V4报告没有shell退出码字段。C的NAV_EXIT=1来自用户终端验收，不回写历史报告。
- C租约保存的实际服务发现包括`set_pose`/blocking（Pose→Boolean）、`set_pose_vector`/blocking（Pose_V→Boolean）、create/remove的blocking端点。这是实际发现证据，不依赖名称猜测。
- set_pose是位置命令，不保证两次调用之间存在物理运动；本实现完全不调用它。使用本机Gazebo8.15的System/Link接口，在每个物理PreUpdate施加速度，在PostUpdate发布观测。连续机制来自物理积分，后续D2/D3采样结果见上述实验结果，结论受采集窗口与接触证据限制。

## 架构与兼容

`trajectory.py`从正式v1几何推导路线；`native/BoundedMotion.cc`及`BoundedProfile.hh`实现速度控制、边界和看门狗；`streams.py`读取带仿真时间的Gazebo Pose_V及租约专属状态；`capture.py`、`eval_observer.py`、`metrics.py`负责只读观测和指标；`moving.py`及`evaluate.py`是CLI。

moving直接调用原`lifecycle.run_lease`，仅增加可选motion分支/回调；没有第二套租约协议。仍使用lease_version=1.0、ea_stage61_UUID实体名、同一个flock、create一次、所有权/ID/会话校验及Scene+SDF双重清理证明。motion是可选扩展字段，老租约和Stage6.1 CLI继续可用。动态模型的所有权检查允许有界位姿变化，仍严格检查collision、visual、token及插件参数。

没有改动Planner、语义、V4、Recovery、v2 Plan ID算法、场景资产或旧日志。运动插件是新建测试实体的局部插件，正式Inspection World v1地图、SDF、Manifest不变。

## 路线、安全与停止

- 仅支持当前默认world、turtlebot3_waffle及Inspection v1。路线中心x从新鲜真实`/plan`样本取得，限制在由分隔墙导出的15.5…17.3；横穿y=25.5…26.5，墙内表面为25和27。盒体0.4×0.4×1.2m，整个扫掠区逐格验证空闲。
- 生成前核对真实路径、当前RUNNING任务、地图与正式世界，检查机器人距**整个扫掠区**的距离。保守余量覆盖既有机器人最大0.5m/s、所选障碍速度和2.5秒预算，拒绝近距离生成。
- speed范围0.05…0.3m/s，duration为2…30仿真秒，另有60墙钟秒上限。cross单程到界自然结束；shuttle往返，中途将速度上限减半。常规速度爬升/减速上限0.4m/s²；紧急停止直接置零，不宣称符合该加速度上限。
- 动态实体有惯性、gravity=false、真实collision及不透明visual，不用Actor/Marker替代碰撞体。物理更新使用Link.SetLinearVelocity，不设置Pose。激光是否实际检测必须在现场检查scan/代价地图。
- Gazebo的Link速度命令会覆盖该步外力；本工具是受控速度测试物体，不是逼真的受力行人模型。碰撞几何仍参与物理计算，但不能据此承诺碰撞响应的生物真实性；需依靠净距保护、Contact证据及人工停止约束实验。
- 插件默认零速度，只有收到租约专属heartbeat才运动。0.5墙钟秒失联、时间倒退/单步>0.05仿真秒、位置跳变、偏离路线/姿态、机器人几何净距<0.35m或可用Contact异常都会锁存停止。没有重新启动已停止实体的指令。
- Python持续核对Gazebo位姿、ROS/Gazebo仿真时钟、采样质量和原服务会话；数据陈旧、跳变或状态不可确认则停止心跳、发送stop并进入原清理流程。流程结束还要求至少观察到0.05m真实位移；速度命令ACK不能替代运动证据。
- 0.35m守卫是测试保护，会主动截断部分接近工况，必须写入诊断，不能当Nav2避障成果。守卫只停止障碍，不取消机器人目标。
- 暂停仿真、服务/磁盘卡顿、严重降速或过高RTF可能触发保守停止。不能为了让测试“通过”取消时钟/新鲜度检查。

## 指标定义和不可测项

JSON包含plan_id/run_id/provenance_id、资产身份、租约路径/ID/实体ID、原始样本、轨迹、路径与BT事件。

| 项目 | 实际支持与边界 |
|---|---|
| 机器人位姿/速度 | 同一Gazebo世界Pose_V的真实位姿；20Hz目标采样，速度为有效区间的差分估计。另记录新鲜且帧合法的/odom速度；不将odom坐标混入map距离。 |
| 障碍轨迹 | 同一Pose_V中的实体位姿/尺寸及原生插件状态。原生状态与Gazebo读回相近时刻须一致。D1无障碍，相关值为null。 |
| 最小净间距 | 机器人使用正式Nav2配置的0.22m保守圆包络，障碍使用可旋转矩形，计算带符号几何距离。负值表示可能重叠，不代替Contact证据。不是机器人精细网格模型的精确间距。 |
| TTC | 固定盒体方向、瞬时匀速相对平移的圆/盒扫掠模型；无可靠速度、无接近/不会相交时null（NOT_APPLICABLE）。不能预测转向或速度变化后的真实碰撞时刻。 |
| 时间/长度 | 观测窗口的仿真耗时和有效区间内机器人轨迹长度；整轮/逐站V4墙钟耗时另列。窗口通常不覆盖整轮，full_navigation_coverage始终NOT_ASSERTED。 |
| 路径/重规划 | 记录/plan每次观测发布的点、长度、哈希和同任务路径变化数。它们是代理，可能含裁剪或小抖动；实际Planner调用次数null。若BT日志可用，另计ComputePath动作进入RUNNING次数，仍只代表观测到的动作。 |
| 恢复 | BT中明确恢复节点的RUNNING转换与原始事件独立保存；V4 CONTINUE/RETRY/HALT只从V4报告统计。无BT事件时NOT_MEASURED，不能报告Nav2恢复0次。自定义节点可能仅保留在原始事件。 |
| Contact | 插件尝试读取所属ContactSensorData，只有时间新鲜且实体ID可解释的真实数据才作为部分证据。正式v1未显式加载Contact系统，预期可能NOT_MEASURED；不为此改正式世界。整体零碰撞结论始终UNKNOWN；正样本数也不是唯一碰撞事件数。 |
| 传感器延迟 | NOT_MEASURED/null。消息到达时间、Pose_V与scan时间差不等于可验证的传感器端到端延迟。 |

仿真时间用于几何差分、TTC、轨迹与运动时限；monotonic墙钟用于新鲜度、心跳、进程超时。BT事件内部时间在已安装消息定义中为wall/unspecified，保留原值但不混入仿真时差。

输入必须帧合法、时间递增；ROS/Gazebo时钟差不得超过0.5s。采样区间>0.25仿真秒或>0.5墙钟秒、位置跳变等标记INCOMPLETE并停止本次运动。所有长度/最小值只代表实际采样，无法排除采样间未捕捉的接触。由于故障/中断产生的报告必须保留，不强制改为成功。

## 准备：只编译，不启动仿真

```bash
cd ~/embodied-agent
source /opt/ros/jazzy/setup.bash
cmake -S reliability/native -B build/stage62_heartbeat -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/stage62_heartbeat -j2
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider

PLAN="$PWD/plans/inspection_qwen_probe_01/plan.json"
REFERENCE="$PWD/logs/task_v4_c45f19509bfc47db849fbfe3233dbfc2.json"
python3 -B -m reliability.moving --plan "$PLAN" --run-report "$REFERENCE" \
  --journal /tmp/stage62_preview_lease.json --metrics /tmp/stage62_preview_metrics.json
```

最后命令是离线Dry Run，不创建这些文件或构造ROS/Gazebo节点。不自动编译/下载/安装插件；Apply要求已经存在的本机库，并将其SHA256写入新租约。需要重新编译时先结束所有旧实验，不替换正在加载的库。

原生通道保留有界就绪握手，当前使用HeartbeatV2协议及`build/stage62_heartbeat/libea_bounded_motion.so`，旧库不覆盖。前一版加载证据见[通道修复说明](MOTION_CHANNEL_HANDSHAKE.md)，当前调度、健康凭据、进度文件及诊断见[D3心跳修复](D3_HEARTBEAT.md)。

在操作者现有、已定位的独立Inspection会话中，先手动只读检查：

```bash
python3 -B -m reliability.obstacles probe --live --plan "$PLAN" --run-report "$REFERENCE"
ros2 topic list -t
```

必须确认无Stage6.1残留、无UNKNOWN未处理租约，世界与/map匹配。服务发现/接口类型不符时停止，不猜名称，不启动另一套共享TF/Action的栈。本受限开发环境不连接实时Gazebo图。

## D1：无新增障碍的定量基线

终端1（每组都新开一次独立导航；唯一发送导航目标的命令，由用户执行）：

```bash
cd ~/embodied-agent
source /opt/ros/jazzy/setup.bash
PLAN="$PWD/plans/inspection_qwen_probe_01/plan.json"
python3 -B -m planner.execute_plan --plan "$PLAN" --execute
NAV_EXIT=$?
printf 'NAV_EXIT=%s\n' "$NAV_EXIT"
```

终端2（导航开始后尽早运行；同样先source和设置PLAN）：

```bash
RUN_REPORT=$(python3 -B -m reliability.obstacles find-run --plan "$PLAN") || exit 1
TAG=$(date +%Y%m%d_%H%M%S_%N)
METRICS="$PWD/logs/reliability/D1_${TAG}_metrics.json"
python3 -B -m reliability.evaluate --plan "$PLAN" --run-report "$RUN_REPORT" --output "$METRICS"
python3 -B -m reliability.evaluate --plan "$PLAN" --run-report "$RUN_REPORT" \
  --output "$METRICS" --wall-seconds 300 --record
```

仅观察，不生成实体。轮次状态与V4耗时从报告取得；采样开始之前的轨迹不补造。保存退出码和完整最终报告。窗口数据与D2/D3比较时须对齐第3站及共同时间窗口，不能把完整D1长度与D2短窗口直接比较。

## D2：单程横穿

重新在终端1明确运行上述稳定导航入口。终端2应在第1或第2站RUNNING时尽早启动工具，先建立只读订阅，再等待第3站进入RUNNING；不会启动导航或要求Nav2重新规划。find-run只找到当前RUNNING报告，不表示已经是第3站。路径观测修复与诊断见[说明](D2_PATH_OBSERVATION.md)。

```bash
RUN_REPORT=$(python3 -B -m reliability.obstacles find-run --plan "$PLAN") || exit 1
python3 - "$RUN_REPORT" <<'PY'
import json,sys
d=json.load(open(sys.argv[1]))
print([(t['id'],t['status']) for t in d['tasks']])
PY
TAG=$(date +%Y%m%d_%H%M%S_%N)
LEASE="$PWD/logs/reliability/D2_${TAG}.json"
METRICS="$PWD/logs/reliability/D2_${TAG}_metrics.json"
python3 -B -m reliability.moving --plan "$PLAN" --run-report "$RUN_REPORT" \
  --pattern cross --speed 0.15 --duration 15 --target-step 3 --journal "$LEASE" --metrics "$METRICS"
python3 -B -m reliability.moving --plan "$PLAN" --run-report "$RUN_REPORT" \
  --pattern cross --speed 0.15 --duration 15 --target-step 3 --wait-task-seconds 120 \
  --journal "$LEASE" --metrics "$METRICS" --apply
```

观察真实实体沿y单程运动、scan/局部代价地图、机器人减速/等待/绕行和路径变化。到界可早于15仿真秒结束并清理；任务变更或导航终止也提前释放。路径不交叉、距离不足或阶段错误时拒绝，重新开展独立轮次，不手填任意坐标。

## D3：往返与速度变化

再开一次独立导航，在第1或第2站RUNNING时重新find-run并提前启动只读观测/等待。不可复用D2的run变量或已存在的日志文件。

```bash
RUN_REPORT=$(python3 -B -m reliability.obstacles find-run --plan "$PLAN") || exit 1
TAG=$(date +%Y%m%d_%H%M%S_%N)
LEASE="$PWD/logs/reliability/D3_${TAG}.json"
METRICS="$PWD/logs/reliability/D3_${TAG}_metrics.json"
python3 -B -m reliability.moving --plan "$PLAN" --run-report "$RUN_REPORT" \
  --pattern shuttle --speed 0.20 --duration 25 --target-step 3 --journal "$LEASE" --metrics "$METRICS"
python3 -B -m reliability.moving --plan "$PLAN" --run-report "$RUN_REPORT" \
  --pattern shuttle --speed 0.20 --duration 25 --target-step 3 --wait-task-seconds 120 \
  --journal "$LEASE" --metrics "$METRICS" --apply
```

路线有界往返，12.5仿真秒后速度上限降为0.10m/s；若导航更早结束或守卫触发，可能观察不到完整变化，必须记录实际窗口，不伪称完成工况。每组建议至少独立重复3次，记录初始AMCL/物理位姿、任务时机、速度、ROS/Gazebo版本和插件哈希；不自动执行批量导航。

## 关联最终V4结果（离线，不重放导航）

D2/D3常在导航尚未结束时完成观测，原指标报告保留当时状态。等待终端1结束后，明确生成一份新报告关联最终V4结果：

```bash
python3 -B -m reliability.evaluate --plan "$PLAN" --run-report "$RUN_REPORT" \
  --from-evaluation "$METRICS" --output "${METRICS%.json}_final.json"          # Dry Run
python3 -B -m reliability.evaluate --plan "$PLAN" --run-report "$RUN_REPORT" \
  --from-evaluation "$METRICS" --output "${METRICS%.json}_final.json" --record # 只写新文件
```

验证身份及最终报告状态，保留原观测窗口、当时V4状态和输入报告SHA256，不扩充采样或重写旧文件。任何导航失败/UNKNOWN均保持真实值。无法测量或覆盖不足时仍然不足。

## 停止、清理与UNKNOWN

1. 障碍终端Ctrl+C或SIGTERM：停止心跳、发布锁存stop，尽力读回零速度**命令**（不是机器人静止证明），随后原租约清理并验证实体消失。异常停止和无法确认停止都产生诊断；实体删除确认才是清理依据。
2. 导航仍在独立终端运行；要停止机器人，应在导航终端Ctrl+C，让V4执行原取消/确认流程。障碍工具不发送机器人控制或全局取消。
3. SIGKILL/宿主崩溃无法finally。插件失联后在物理更新中停止，操作者须重新检查，不把看门狗当作已验证清理。使用**同一套Stage6.1工具**：

```bash
python3 -B -m reliability.obstacles cleanup --journal "$LEASE"          # Dry Run
python3 -B -m reliability.obstacles cleanup --journal "$LEASE" --apply
python3 -B -m reliability.obstacles probe --live --plan "$PLAN" --run-report "$RUN_REPORT"
```

活跃租约持有同一进程锁，先中断原持有者，不抢锁、不删除锁文件。会话改变、ID复用、所有权/路线不匹配或无法确认移除，保留UNKNOWN及退出码4，禁止开展下一轮，人工核查Scene、生成SDF和Gazebo GUI；不修改日志来伪装CLEAN。必要时GUI暂停物理，确认导航目标已经取消后再恢复/清理，或结束专用仿真。暂停期间服务命令可能排队，不能直接以ACK作为终止证据。

## 本次验证与待验收

原582项全部回归通过，新增106项，共688通过、0失败。完整原生插件本机CMake编译成功；独立C++控制内核测试执行了单程、往返、心跳/接触/越界/时钟等分支。动态SDF样例通过`gz sdf -k`，仅静态解析。未执行D1/D2/D3。

待真实验收：插件加载、私有状态/控制端点发现、无heartbeat时不动、失联/stop锁存、实际连续位移与速度、墙体/机器人保护、激光及代价地图检测、清理双读回、指标采样质量、BT日志实际可用性和Contact是否可用。需录制scan/TF/clock/path/局部代价地图及视频并保留真实结论；不存在Contact证据时不能签署零碰撞。安全守卫不保证对外部传送、任意延迟、恶意本地修改或机器人不制动等所有情形安全。
