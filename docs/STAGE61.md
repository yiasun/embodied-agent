# Stage 6.1：Gazebo障碍注入与可靠性测试基础设施

本阶段只实现代码、Mock测试及静态SDF校验。没有启动仿真、调用模型API、注入/移动/删除真实实体或执行导航。

## 现状审计

最新报告：`logs/task_v4_5aaa38b2c06d40c2b7a5e4898e592e89.json`。

- 四个任务均为SUCCEEDED/goal_reached，耗时33.45、33.62、32.97、33.62秒；每步首次成功、恢复决策均为CONTINUE。
- plan_id=`c14287a68250a91862a14b1618ca73d63efa2e7a011484cc8c18849433ec2e7e`，算法trusted-plan-sha256-v2；完整计划、资产版本、注册表坐标、报告有序目标校验通过。
- provenance_id=`5ec24cfefe7e94ca0160bf75a48881d1c5b964501c8cfa6336cbd47a33c807d0`，与`plans/inspection_qwen_probe_01/provenance.json`关联一致。
- 来源qwen/qwen-plus，request_id=`chatcmpl-9309c46e-d628-92be-aa5b-044273fc8a25`，Token=269/49/318，耗时1.78899秒。
- 报告没有exit_code字段。当前代码映射SUCCEEDED→0，但没有历史shell退出码的直接记录；不能据报告补造历史退出码。
- 现有静态地图、世界和Manifest哈希通过；Navfn/MPPI、Obstacle/Voxel层、激光订阅及V4有限重试/取消逻辑保留。本阶段没有改动这些模块。

接口证据：本机已有gz.transport13、gz.msgs10，已读取Node.request实现及EntityFactory、Entity、Scene、Pose_V等消息描述符；安装库导出符号也明确给出async/blocking的EntityFactory→Boolean创建与Entity→Boolean删除签名。
安装库及历史服务器日志确认create/remove/set_pose、scene/info和generate_world_sdf接口体系，创建/删除有async和blocking变体。
本会话`gz service -l`因getifaddrs网络接口限制失败，**没有取得当前实时服务图**。工具运行时会重新发现、核验请求/响应类型和唯一发布者；缺服务或类型不符直接停止，不照猜测地址发送请求。

## 文件与边界

新增`reliability/`：

- `safety.py`：可信计划/Trace核验，地图几何、真实路径及机器人位置约束。
- `gazebo.py`：延迟加载Gazebo绑定，服务发现、实体/SDF/位姿查询、一次创建和精确ID删除。
- `observer.py`：只读订阅ROS `/map`和`/plan`；不创建Action客户端。
- `lifecycle.py`：租约、读回核验、异常/中断清理和UNKNOWN处理。
- `obstacles.py`：默认Dry Run CLI、进程锁、显式Apply、独立清理和活动报告查找。

新增离线测试及本文、审计JSON；只修改README和pyproject包发现。原Planner、Provider、V4、Action状态机、Recovery、正式资产、旧测试和历史报告保持原样。
没有新增pip/ROS依赖，运行时使用现有ROS安装环境中的Gazebo Python/protobuf绑定。

## 安全位置的依据

不是直接输入任意x/y，也不把离线合成路径当实际Nav2路径。

1. 从正式几何的south_partition_mid、north_partition_mid及south_room_divider推导西侧中央通道：y=25…27，允许的x带为15.5…17.3。
2. Apply必须读到新鲜的ROS `/plan`，坐标系为map，末端与当前RUNNING任务目标一致，路径起点接近Gazebo中机器人当前位置。
3. 从真实路径的x样本中选择、对齐0.05m网格，障碍中心y取该通道剖面的中心26。障碍必须真实与当前路径相交；不在允许区域就拒绝，不勉强找别的坐标。
4. 每个障碍占据的所有格子必须在正式地图空闲区。机器人位姿使用新鲜Gazebo `/world/<world>/pose/info`，而非可能偏移的AMCL位姿。
5. 最后发送创建前再次核对机器人距离和当前任务。最低净距为机器人半径+1m余量+配置最大速度×2.5秒（本配置2.47m），用于覆盖观测/请求延迟；过近、陈旧或非有限位姿都拒绝。

B使用0.5×0.5×1.2m盒体；C使用0.4×1.9×1.2m通道屏障。两者均为static实体，含一致的collision和不透明visual，供物理碰撞和GPU激光渲染，不是RViz Marker。
C还会在当前机器人/目标之间做静态栅格连通性检查，确认该屏障确实切断0.3m净空路径；不能切断则拒绝。
示例几何位置可能为(16.4,26,0.6)，但这只是静态样例，实际坐标必须由运行时路径选择并写入租约。

## 创建与撤除协议

- 默认Dry Run不构造Gazebo/ROS节点，不写租约、不发服务请求。
- Apply要求inspection_world的v2可信计划、匹配的RUNNING报告和合法Provenance；不允许跳过现有Plan ID/场景检查。
- 运行前核验Gazebo正式静态模型、碰撞/视觉尺寸及位姿。额外实体、残留测试障碍或错误世界会阻止创建。
- 全局flock拒绝并行注入；UUID实体名、allow_renaming=false防止重复创建。每个租约只发一次create，不重试不确定请求。
- 创建前持久化日志；之后按Scene与生成世界SDF读回，核验名称、ID、静态状态、位姿、碰撞/视觉几何和随机所有权标记。布尔ACK不算成功证据。
- 删除前再次核验以上信息及服务发布者会话摘要；只删除精确MODEL实体ID。不按用户提供任意名称删除，不清空世界，不操作机器人。
- 正常租约到期、导航结束、Ctrl+C、SIGTERM或异常都会尝试撤除；成功必须从Scene和生成世界SDF两处读回实体消失。
- create超时且实体从未读到时，即使当前看不到模型，仍记录UNKNOWN，防止把延迟执行的创建误判为已清理。
- 无法确认删除、所有权不符、实体ID变化或服务器会话改变均返回UNKNOWN（退出码4），不盲删。普通失败1，中断且清理确认130，正常租约完成且清理确认0。

日志位于新的`logs/reliability/lease_*.json`或显式指定的新文件。包含UTC请求/确认时间、模式/租约时长、位置/尺寸、实体ID、路径摘要与相交样本、plan_id/run_id/provenance_id、服务契约及清理结果。
不记录API Key、环境变量或服务器任意异常文本。未知实体ID保留null，不伪造0。已有日志文件拒绝覆盖；当前租约文件采用原子更新。

工具exit0只说明租约创建/清理流程完成，**不等于避障测试通过**。

## 准备命令（用户后续手动运行）

保持已有inspection_world会话，不要再启动第二套共享/map、TF与Action名字的栈。本工具不会启动仿真或导航。

```bash
source /opt/ros/jazzy/setup.bash
cd ~/embodied-agent
PLAN="$PWD/plans/inspection_qwen_probe_01/plan.json"
REFERENCE="$PWD/logs/task_v4_5aaa38b2c06d40c2b7a5e4898e592e89.json"

# 纯离线Dry Run，可使用已完成报告作为身份参考；不要求报告仍在运行
python3 -B -m reliability.obstacles inject \
  --plan "$PLAN" --run-report "$REFERENCE" --mode temporary

# 先只读发现接口、确认没有额外模型。当前受限会话未执行成功此步骤
python3 -B -m reliability.obstacles probe --live \
  --plan "$PLAN" --run-report "$REFERENCE"

# 独立检查Gazebo实际名字/类型（只读；以probe实际发现为准）
gz service -l
gz service -i -s /world/default/scene/info
```

Apply期望发现的类型是EntityFactory→Boolean、Entity→Boolean、Empty→Scene、SdfGeneratorConfig→StringMsg及Pose_V主题。
优先选择已发现的blocking创建/删除端点，未发现就用已发现且类型正确的async端点；都没有则拒绝。
`set_pose`等移动服务只做发现审计，Stage6.1没有移动命令。

## A：无新增障碍的正常基线

1. 只读probe确认正式场景匹配且没有测试障碍；等待scan/局部代价地图稳定，人工确认没有残留占据。
2. 用户在导航终端明确执行：

```bash
python3 -B -m planner.execute_plan --plan "$PLAN" --execute
NAV_EXIT=$?
printf 'NAV_EXIT=%s\n' "$NAV_EXIT"
```

3. 保存本次新V4报告及终端退出码，四任务SUCCEEDED、顺序/坐标/Trace一致才算此轮基线通过。
本工具不会执行上述命令，也不修改历史成功报告来代替新基线。

## B：临时出现、随后撤除

在导航终端再次由用户手动运行同一稳定执行命令。在另一个终端设置PLAN相同路径：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/embodied-agent
PLAN="$PWD/plans/inspection_qwen_probe_01/plan.json"
RUN_REPORT=$(python3 -B -m reliability.obstacles find-run --plan "$PLAN")
```

find-run只读查找唯一合法RUNNING报告；没有或多个时失败，不猜“最新文件”。务必检查命令返回成功，不能继续使用上一次shell变量。
建议固定在第3任务（仓储间→机房）开始时进行对比；太晚接近测试位置或任务阶段不符会拒绝，需新建独立实验再尝试。

```bash
LEASE="$PWD/logs/reliability/B_$(date +%Y%m%d_%H%M%S).json"
python3 -B -m reliability.obstacles inject \
  --plan "$PLAN" --run-report "$RUN_REPORT" --mode temporary \
  --target-step 3 --hold-seconds 8 --journal "$LEASE"           # Dry Run

python3 -B -m reliability.obstacles inject \
  --plan "$PLAN" --run-report "$RUN_REPORT" --mode temporary \
  --target-step 3 --hold-seconds 8 --journal "$LEASE" --apply   # 修改仿真；本阶段未运行
```

观察红色真实盒体、scan距离变化、局部代价地图标记、路径重规划、速度与等待。8秒后或导航结束时自动撤除并验证。
撤除后观察代价地图清空和导航是否继续；保存租约、报告和录像。只有实际观察才能判断避障或恢复行为。
这是环境对象出现/消失，不能称为连续移动障碍避让。

## C：持续阻塞与安全终止

单独新建一次人工导航运行，重新取得唯一RUN_REPORT。在第3任务开始时：

```bash
LEASE="$PWD/logs/reliability/C_$(date +%Y%m%d_%H%M%S).json"
python3 -B -m reliability.obstacles inject \
  --plan "$PLAN" --run-report "$RUN_REPORT" --mode persistent \
  --target-step 3 --hold-seconds 150 --journal "$LEASE"          # 先Dry Run

python3 -B -m reliability.obstacles inject \
  --plan "$PLAN" --run-report "$RUN_REPORT" --mode persistent \
  --target-step 3 --hold-seconds 150 --journal "$LEASE" --apply
```

150秒是显式测试上限，不会因默认运行工具而开始。当前稳定入口每次导航60秒、最多重试1次；Nav2也可能更早ABORT，不能强求某个失败码。
观测并分别记录：

- Nav2 Planner/Controller/BT内部的清图、等待、旋转、后退或重规划，属于Nav2自身行为。
- V4只有TIMED_OUT且确认取消后才可能RETRY；其他失败HALT，UNKNOWN禁止后续任务。该策略未修改。
- 后续任务应SKIPPED，退出码非零，最终Goal确已终止。UNKNOWN不能算“安全终止已验证”。

若报告先结束，工具会提前清理障碍，不额外修改机器人状态。若Nav2意外成功，应保留实际结果分析绕行或测试条件，不能伪写失败/恢复结论。

## 停止、显式撤除与异常恢复

- 正常停止：先在导航终端Ctrl+C，让原入口/V4执行有界取消并写报告；在障碍终端Ctrl+C，工具finally尝试清理。不要把清障碍当作取消导航。
- 立即风险：操作者可先在Gazebo GUI暂停物理，再处理导航取消。暂停可能让实体命令排队，清理因此UNKNOWN是正常的保守结果；确认导航已终止后再恢复物理并清理，或结束专用测试仿真，不在Goal未知时直接恢复运行。
- SIGTERM也尝试清理；SIGKILL、进程/主机崩溃无法运行finally。此时使用已知LEASE文件：

```bash
python3 -B -m reliability.obstacles cleanup --journal "$LEASE"          # Dry Run
python3 -B -m reliability.obstacles cleanup --journal "$LEASE" --apply  # 验证所有权后撤除
```

仍活跃的租约持有全局锁，另一个cleanup进程会拒绝抢占；先正常中断租约持有者，再执行独立清理。工具不会自动向其他进程或机器人发停止信号。
已清理的租约可重复核查，不会删除别的实体；新文件不能覆盖旧租约。
Gazebo重启会改变服务会话，旧租约拒绝删除新世界的同ID实体。UNKNOWN必须保留并人工检查，不能改成CLEAN充当成功。

## 观测材料与判据

可在已有工具可用时，先用`ros2 topic list -t`确认实际话题，再手动录制/观察`/clock`、`/scan`、`/plan`、`/local_costmap/costmap`、`/local_costmap/costmap_updates`、`/tf`、`/tf_static`、`/amcl_pose`及Action状态/反馈。部分话题是隐藏话题，rosbag需对应选项；录制命令不由注入工具启动。
每组保留PLAN、Provenance、V4报告、shell退出码、租约、当前任务号和录像，至少人工确认无碰撞/Goal终态/清理完成，再评价测试结果。
不只用“最终SUCCEEDED”推断无碰撞，也不从Nav2日志推断Agent具有新增自主恢复能力。

## 离线验证与限制

本阶段新增71项，原511项保持通过，总计582项通过、0失败（最终回归）。SDF样例通过本机gz sdf -k，仅解析未注入。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider
```

限制：

- 当前受限会话无法完成实时Gazebo发现；Create/Remove/Scene/SDF读回的真实端到端连通性尚未测试。严格读回与实际服务形态不一致会失败关闭，不用Marker或布尔ACK伪装成功。
- 安全距离基于正式Nav2 DiffDrive速度参数、fresh pose与延迟预算，不是对并发外部传送机器人/改速或任意网络延迟的形式化安全保证。
- 只支持当前默认world/机器人命名、四室场景和几何导出的西侧通道；没有任意坐标、任意模型删除或多个并行注入。
- 路径不经过该窗口、机器人太近、报告已结束或多RUNNING报告均拒绝；不自动调度/启动导航来制造条件。
- 动态障碍不写入正式/map，因此不会改变v2资产哈希；其环境变化证据在独立租约中，正式SDF/地图/Manifest不变。
- 服务ACK、读回和日志间存在分布式不确定性；UNKNOWN需要人工处置，不能保证崩溃后自动清理。
- 日志及所有权标记防止误操作，不是对有本机写权限者的认证机制。

Stage6.2建议：在本阶段真实短租约冒烟验证后，加入受控连续轨迹/速度与时间同步、停止与边界检查，验证真实运动而非瞬移；再增加TTC/最小距离、碰撞、延迟与路径变化度量，仍将Nav2恢复和Agent级决策分开评价。

2026-11-09补记：用户已完成A/B/C真实仿真验收；B两份租约均CLEAN且对应四点成功，C租约CLEAN且第3站失败/HALT、第4站SKIPPED。历史文件保持原样。Stage6.2复用本租约协议，代码与离线验证、真实待验收事项见[Stage6.2](STAGE62.md)。上文“尚未执行”描述的是6.1初次开发时的状态。
