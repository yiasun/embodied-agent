# D3 心跳调度、监督门禁与诊断

本轮仅代码、Mock、编译与静态验证。没有启动Gazebo、发送导航目标、调用模型API或注入真实障碍。

## 审计结论

保留的D3租约`logs/reliability/D3_20261109_141944_029629433.json`及metrics确认：HEARTBEAT_EXPIRED、CLEAN、LATCHED_ZERO_COMMAND；177个样本、9.519仿真秒、最大采样间隔0.057秒、无丢弃旧位姿、issues为空，几何净距最小4.607m。往返一次的观察成立，但不算完成25秒工况。

旧Python在唯一主循环中依次执行Capture.tick（含关键任务文件读取与校验）、send、Gazebo发现、多文件原子写入/fsync、报告构造和checkpoint。任何同步耗时叠加都可能让相邻心跳超过500ms；仿真采样间隔小不能证明这些墙钟操作没有长停顿。旧记录缺少发送/接收耗时，无法唯一认定是哪项引起本次故障。日志不回填推测值。

修改前快照：`.snapshots/d3_heartbeat_20261109T063541Z/`，包含源码、旧插件及SHA256清单；历史日志、计划保留原地。未修改Git元数据。

## 保活结构

**没有独立心跳线程。只有完成本轮安全监督的前台主控制器才能发心跳。**

| 工作 | 执行位置与约束 |
|---|---|
| 任务、真实位姿、运动状态、安全检查 | 前台；capture.tick单次预算0.2墙钟秒；采样/原生消息年龄≤0.25秒；任务必须仍为目标RUNNING，几何净距≥0.35m。既有身份、地图、frame、时间和质量校验保留。 |
| 心跳 | 前台完成健康检查后发送。发送间隔与调用耗时记录，间隔≥0.5秒或发送调用>0.1秒则失败关闭。没有后台保活兜底。 |
| 世界会话发现 | 独立只读Python进程及Gazebo node，后台线程只等待IPC；每次完成后至少0.2秒再查询，子进程响应预算0.9秒、总任务预算1秒。主线程只检查结果/截止时间。会话证明年龄以查询开始时刻计，最多2.5秒；超时、变化或过期停止保活。 |
| 进度日志与checkpoint | 单个后台任务，无无限队列；总预算2秒，已有任务未完成时跳过新的进度快照并统计。JSON序列化/fsync不在心跳路径。 |
| 最终停止与清理 | 先停保活/发stop、确认零命令，再停后台任务、写最终观测并进入原租约清理。后台发现节点独立，不与主节点清理共享bindings写入。 |

主控制器卡在capture、文件读取、异常或失去CPU调度时，不会再发心跳。后台线程只查询元数据或写进度文件，不操作实体、不控制机器人。关键监督检查不能移出主线程并以“心跳正常”掩盖其失联。

核对的[Gazebo Transport 13.6 Python绑定](https://raw.githubusercontent.com/gazebosim/gz-transport/gz-transport13_13.6.0/python/src/transport/_gz_transport_pybind11.cc)对service_list、service_info、topic_info没有显式GIL释放。因此仅把这些调用放到另一个Python线程不足以隔离阻塞。`probing.py`使用spawn独立解释器和有界Pipe响应，隔离GIL及SDK共享锁；子进程仅调用discover，退出时最多终止自己创建的只读helper，绝不终止Gazebo、Nav2或V4。Mock测试还以子进程内持有GIL的原生调用验证父进程仍可调度。

初始化时的就绪握手、首个会话检查及WAL仍在首个心跳前同步完成；在审计落盘之前不开始运动。关键capture读取如果长期阻塞，安全结果仍应是停止，不保证所有低性能环境都能完成25秒。

## 防止延迟旧心跳续期

只停止发送还不足以约束传输队列里已发出的旧消息。V2使用私有StringMsg格式`heartbeat:<发送序号>:<已校验原生状态序号>`；Python的send API仍是heartbeat/stop，stop无需凭据。

插件保存最近64个状态的**自身steady-clock发布时间**。只授权序号递增、引用已发布状态且凭据年龄≤0.25秒的心跳；watchdog仍为0.5秒，计时基准使用该状态的发布时间，不能用延迟抵达时刻延长授权。

前台发送时绑定的是本轮健康检查确认过的确切状态及接收时间；卡死后新到的状态不能替换旧监督凭据。发送卡住、旧包延迟、重复或未知凭据均不能恢复已锁存停止的运动。无需跨进程时钟对齐，不用仿真时间冒充墙钟时间。

`BoundedProfile.hh`、0.35m净距、速度和加速度限制保持原样。500ms是相对最后有效原生凭据的授权阈值，可能比最后一次接收心跳后500ms更早停止，绝不更晚续期。物理命令在首个可调度PreUpdate置零；若整个Gazebo物理线程/宿主暂停，不能宣称任意负载下具有严格500ms硬实时调度保证。恢复物理前会先检查过期；本轮没有实测硬实时停止延迟。

## 记录什么

Journal和最终metrics附加`controller_diagnostics`：

- 每次capture_tick、gazebo_discovery、json_writer、checkpoint、heartbeat_send、heartbeat_send_interval、io_poll、快照准备等的单调墙钟耗时；累计次数、成功/失败、总耗时、最大值和最近512条单次事件。最终checkpoint另外标记，避免把停止后的写入算成保活阻塞。
- 查询/持久化截止时间、会话证明年龄、跳过的快照数、关闭时未完成任务及最后错误。不可抢占的原生元数据或磁盘调用可能在停止后才返回；它们不具备发心跳权限。
- motion_trace保留Python接收单调时刻；插件状态增加heartbeat_received_count、heartbeat_authorized_count、heartbeat_rejected_count、heartbeat_age_wall_s、heartbeat_authorization_age_wall_s、接收间隔/最大间隔、PreUpdate墙钟间隔/最大间隔。尚无观测时为null，计数0是实际计数。

判读时：Python发送间隔大支持前台调度阻塞；发送正常但原生接收间隔大可能是传输/接收线程调度或丢包；凭据拒绝增多说明旧状态、延迟或重复；PreUpdate墙钟间隔大则支持物理调度停顿。这些证据可能重叠，不唯一归因。Publish成功不等于已送达；独立进程steady-clock epoch不作同步假设，**单向Transport延迟仍为NOT_MEASURED/null**，不能把两种间隔之差写成精确传感器/网络延迟。

## 进度文件与最终文件

后台只写新实验的`<Journal路径>.progress.json`与`<Metrics路径>.progress.json`，带progress_only、generation、snapshot_mono。CLI在创建实体前用O_EXCL保留这些新文件，拒绝覆盖历史文件。样本/路径/事件列表按不可变已发布记录快照，后续append不会改变旧进度快照。

规范Journal/Metrics由前台在准备及停止/清理后写入，不由进度线程写。即使磁盘调用晚返回，也只能生成带标记的旧进度文件，不能覆盖最终CLEAN。最终验收/离线finalize必须使用规范metrics，不用progress文件冒充终态。

若全部磁盘永久卡死，最终落盘本身仍可能无法完成；本实现不能保证日志一定持久化。但该阶段已经停止发送心跳并尝试stop，磁盘线程不能维持运动。不得把未确认的清理改写为CLEAN，仍沿用原UNKNOWN与人工核查流程。

## 构建与后续人工复测

新原生类名为`embodied::BoundedMotionHeartbeatV2`、协议`ea-motion-heartbeat-v2`，默认独立库路径如下。旧原始/V1插件和历史租约保持原样；清理验证仍兼容旧注册名。新旧协议不静默混用。

```bash
cd ~/embodied-agent
source /opt/ros/jazzy/setup.bash
cmake -S reliability/native -B build/stage62_heartbeat -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/stage62_heartbeat -j2
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider
```

用户后续明确复测D3时，沿用[现有提前订阅/等待流程](D2_PATH_OBSERVATION.md)，使用全新租约和metrics，pattern=shuttle、speed=0.20、duration=25；可显式指定`--plugin "$PWD/build/stage62_heartbeat/libea_bounded_motion.so"`。不放宽检查强求成功。

先用Dry Run检查，再由用户明确Apply。本轮没有执行上述真实实验。停止、Ctrl+C、异常清理仍走原生命周期；后台没有独立心跳，进度线程也不调用清理。独立清理命令不变：

```bash
python3 -B -m reliability.obstacles cleanup --journal "$LEASE"          # Dry Run
python3 -B -m reliability.obstacles cleanup --journal "$LEASE" --apply
```

## 本轮离线结果

原770项覆盖保持通过，新增35项，总计805通过、0失败。包含慢发现、慢writer/checkpoint、阻塞磁盘、capture超时、任务/数据过期、丢包、主循环卡死、延迟旧心跳、监督凭据替换、正常持续心跳、SDK进程GIL隔离、退出错误及晚写入不覆盖终态。C++真实watchdog helper与原运动内核的编译测试覆盖0.499/0.500秒边界、锁存停止及拒绝重放。

待真实验收：V2加载与协议、D3完整25仿真秒、原生接收/授权间隔、后台I/O是否消除现场长尾、实际停止延迟、清理双读回。原故障不能因本轮Mock通过而改写为已完成。
