# Stage 6.2 原生运动通道就绪握手

本文保留V1握手修复的历史设计与验证记录。当前默认库、HeartbeatV2协议和保活调度见[D3心跳说明](D3_HEARTBEAT.md)，不使用旧V1库替代新版。

本轮仅离线修复；未启动Gazebo、发送导航目标、创建真实实体或发送运动心跳。

## 现场证据与结论

历史租约`logs/reliability/D2_retest_20261109_131755_771827992.json`中的实体117创建成功、路径两次SELECTED，随后通道检查失败并CLEAN。本轮不修改该记录。

服务器留存日志`~/.gz/sim/log/2026-11-09T11:17:18.827165370/server_console.log:89`明确出现`Loaded system [embodied::BoundedMotion] for entity [117]`，下一行创建名称与租约相符。历史SDF可从租约确定性重建，其绝对库路径、别名、token、link名字及数值参数与代码一致。发布文档已去除本机用户目录；原始服务器日志不随仓库分发。

Gazebo 8.15的[SystemManager源码](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8_8.15.0/src/SystemManager.cc)在加载日志前调用AddSystemImpl，其中执行Configure；[SdfEntityCreator源码](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8_8.15.0/src/SdfEntityCreator.cc)会为动态模型加载模型插件。这些证据支持实体117已经加载系统并调用/返回Configure，不能证明其没有提前返回。

旧代码在INVALID_CONFIG、MISSING_ENTITIES时先返回，尚未Advertise/Subscribe；CONTROL_SUBSCRIPTION_FAILED也缺少持久化的启动状态。旧Python只查一次端点且丢失具体数量。因此发现传播延迟与Configure提前失败都可能产生同一个错误，无法从历史记录唯一归因。不能据此称Gazebo不支持动态加载，也不能直接宣称只是时序竞争。

## 实现与协议

- MotionChannel默认有3墙钟秒应用层截止时间，最多每50ms检查一次就绪条件；条件满足即可返回，不固定sleep若干秒后盲信成功。原生元数据调用不能被Python抢占，返回后仍检查截止时间，elapsed记录实际耗时；不以阻塞为理由发送心跳。
- 每次核对state发布者和control订阅者数量、实际消息类型。缺失允许等待；重复或错误类型立即失败。收到初始状态后再核对一次端点。
- 唯一且类型正确的state端点出现即可只读订阅，即使control还未出现，也能接收插件启动失败诊断。未就绪不会创建控制发布者或发送任何命令。
- 初始状态必须是`ea-motion-channel-v1`协议，token、entity_id、entity_name严格匹配当前租约；frame必须为gazebo_world。字段必须类型正确、有限、时序递增，接收年龄≤0.25秒。
- Configure状态必须READY/configured=true，且stopped=false、reason=WAITING_FOR_HEARTBEAT、commanded_vy=0、位姿有效并接近已验证生成位置。不能用“端点存在”或“收到任意JSON”代替握手。
- 就绪后再核对原Gazebo会话，持久化启动证据，然后才创建运动位姿采集并按原流程等待新鲜物理观测后发心跳。这样等待期间的积压不会进入本次运动采样。
- 构造失败会注销自己的state订阅，将诊断保留在租约；仍由原run_lease完成唯一所有权、ID、会话核验及Scene/SDF双重清理。没有开始运动时motion_stop保持NOT_STARTED；该值不代替清理证明。

新插件会在Configure输出`[EA_MOTION_INIT]`的BEGIN、READY或FAILED日志，附entity/link/robot ID及固定错误码。有效token的state发布者在数值/实体校验之前建立，PostUpdate即使缺少link也可发布FAILED状态，诊断INVALID_CONFIG、MISSING_ENTITIES、CONTROL_SUBSCRIPTION_FAILED等。非法token无法安全建立正确命名空间时仅在服务器日志记录错误。

0.5秒心跳看门狗、0.35m净距守卫、速度/加速度/轨迹限制没有改变；BoundedProfile.hh保持原样。未修改Planner、V4、地图、路径观测逻辑或D1采集。streams.py中的PoseStream/decode_poses代码保持原样。

## 新库构建与部署区分

新协议要求新库，旧库缺少protocol/configured等字段时明确报NATIVE_PROTOCOL_MISMATCH，不自动兼容成“成功”。为保留旧二进制、区分已加载实例，新库构建到独立目录；moving的默认plugin路径改为下面的新文件。仍支持显式`--plugin`，其他CLI参数不变。

Gazebo的[SystemLoader源码](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8_8.15.0/src/SystemLoader.cc)加载库后按插件名实例化，因此不能仅靠更换文件路径来区分同名实现。新版注册名及C++类名为`embodied::BoundedMotionHandshakeV1`；新租约明确记录motion.plugin_class，SDF与所有权读回必须匹配该类名。历史租约缺少此字段时仍明确映射为旧`embodied::BoundedMotion`，不改写过去的数据。其他未知类名在生成SDF前拒绝。

```bash
cd ~/embodied-agent
source /opt/ros/jazzy/setup.bash
cmake -S reliability/native -B build/stage62_handshake -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/stage62_handshake -j2
ldd build/stage62_handshake/libea_bounded_motion.so
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider
```

本机已经完成上述编译和离线检查，无安装或升级依赖。旧`build/stage62/libea_bounded_motion.so`保持原样；不覆盖活动进程可能仍持有的库。不修改系统目录或正式场景SDF。历史租约按各自原绝对路径验证，历史报告不迁移。

之后由用户按[现有D2命令](D2_PATH_OBSERVATION.md)明确复测；可显式指定：

```bash
# 加在用户原D2命令中，不是独立执行命令
--plugin "$PWD/build/stage62_handshake/libea_bounded_motion.so"
```

工具仍默认Dry Run，只有用户明确Apply才会创建实体。原停止/清理命令不变。

## 失败诊断

Journal和metrics均附加`motion_channel`：phase、尝试次数、真实elapsed、截止时间、主题、实际发布者/订阅者数量、实际类型、最近16次发现快照、last_error、failure、initial_state_received和最后一条合法原生状态摘要。

- MOTION_HANDSHAKE_TIMEOUT：保留具体的STATE_PUBLISHER_MISSING、CONTROL_SUBSCRIBER_MISSING、INITIAL_STATE_NOT_RECEIVED、NATIVE_INITIAL_STATE_STALE或元数据查询错误类型。
- MOTION_CHANNEL_DUPLICATE_ENDPOINT / MOTION_CHANNEL_WRONG_TYPE：不删除或放宽唯一性/类型要求。
- NATIVE_PROTOCOL_MISMATCH / NATIVE_MOTION_IDENTITY_MISMATCH：旧协议或身份错误，不继续运动。
- NATIVE_CONFIGURE_FAILED_*：收到身份合法的原生失败状态，保留原错误码。
- NATIVE_INITIAL_STOP_* / NATIVE_INITIAL_STATE_NOT_IDLE / NATIVE_INITIAL_POSE_MISMATCH：插件已停止、已非空闲或位置不符，失败关闭。
- WORLD_SESSION_CHANGED_DURING_HANDSHAKE：握手后会话已变，不发送心跳；清理沿用原UNKNOWN处理。

last_error表示最近一次观察到的错误，READY时可能仍保留早先端点缺失的瞬态；phase和最终端点快照共同解释。plugin_startup=NOT_CONFIRMED表示缺少有效启动状态，不能把超时写成“确认未加载”。身份/格式错误的消息不作为可信Configure结果。

## 离线验证与未完成事项

原732项覆盖保持通过，新增38项，共770项通过、0失败。新增覆盖延迟注册、无端点、无初始状态、重复/错误类型、身份与协议错误、超时、正常握手、Configure失败、会话变化、失败Journal/metrics关联及新旧插件身份兼容；C++启动参数与错误分支有独立编译测试。既有通道测试的Mock消息更新为新协议，保留原用例。

新原生插件本机编译成功，source ROS环境后ldd没有未解析依赖；历史SDF离线重建通过gz sdf -k。以上均不是运行时运动验收。

待用户真实验证：新绝对路径的库确实加载、Configure输出READY、实际发现时延、初始身份/空闲状态可收到、收到新鲜物理采样后才有首个心跳，以及异常后的停止/清理。未执行新一轮D2，不宣称原生运动通信或连续运动已经真实通过。
