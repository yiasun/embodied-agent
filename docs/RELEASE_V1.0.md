# V1.0 私有GitHub交付记录

目标仓库：`https://github.com/yiasun/embodied-agent`，分支`main`。仅私有交付；未经维护者进一步确认不公开。

现有[Vercel回放Demo](https://embodied-agent-replay-dv5zxlbmp-sun-1ac1.vercel.app/)保留原有部署与访问保护。本轮不创建Vercel项目、不部署、不更改域名或保护，不发布实时机器人控制接口。

## 源码和历史保护

当前项目根目录`.git`是空的只读tmpfs挂载，遮蔽的信息无法在本会话恢复，不能据此断言原项目没有Git历史。没有覆盖、清空、重新初始化该目录，也没有使用不可撤销的Git操作。

修改前快照：`.snapshots/release_v1_20261109T114108Z/`，200个原文件SHA-256清单和源码归档；日志、历史计划保留原地。发布采用独立白名单副本`.delivery/v1.0/worktree/`，该目录被主工程`.gitignore`排除；其Git信息与原项目`.git`互不混用。后续正常开发建议由维护者在新的空目录克隆GitHub仓库，不能把交付副本的`.git`强行覆盖原项目目录。

ROS导航、Planner、V4、安全门禁、Gazebo原生插件、心跳、正式Inspection World v1资产及历史计划/日志不修改。本轮只修改发布文档、排除规则、包版本和离线测试/公开证据复算入口。

## 安全审计与发布边界

- `.gitignore`排除实际`.env`、密钥/证书、原始`logs/`、模型响应/Provenance包、机器审计JSON、缓存、虚拟环境、编译产物、快照、工具私有目录、Vercel本地关联信息与临时大文件。
- 发布使用`tools/release_audit.py`显式白名单，而非`git add .`原工程。待提交路径见[文件清单](RELEASE_FILES.txt)，每个文件的SHA-256与大小另保存在本地`.delivery/`审计清单。
- 保留源码、严格Schema、场景/地图/Manifest、依赖配置、测试、空密钥`.env.example`、三个纯计划示例、原始分析图及复核图、已脱敏真实回放JSON。
- 原始日志含本机路径、租约所有权/会话信息，不上传；不通过修改原始报告来脱敏。公开数据已去除API Key、原生token、lease_id、entity_name、机器路径与模型request_id等字段。
- 当前执行会话可见的环境中没有发现凭证变量；模板密钥为空。对62个本地计划/日志/环境模板进行扫描，发现41处本机目录路径匹配，全部位于未发布的历史材料。嵌套协议所有权字段另做结构检查，不把这些凭证当作普通可公开日志。
- 对待上传文件再次扫描私钥头、常见云API/GitHub凭证、URL内嵌密码、非空模板密钥、机器用户路径、符号链接及大文件；结果不输出匹配值。只说明本次模式检查未命中，不承诺能发现所有未知格式的秘密。
- `.gitattributes`固定文本LF，并禁止对Manifest绑定资产自动转换换行，避免跨平台签出改变场景字节身份。

审计只覆盖当前可见文件和当前执行环境，不能检查其他终端/宿主的环境变量。真实密钥不进入报告、提交信息或终端输出。

## 离线复现与测试

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider
python3 -B -m replay.public
python3 -B tools/release_audit.py --report .delivery/security-audit.json
```

本地完整工程：**881通过，0失败**。新增公开证据复算、发布白名单/脱敏/排除规则测试，并保留原861项回归。

不含私有历史报告的交付副本：**879通过、2跳过、0失败**。跳过项分别是早期真实API历史哈希关联、完整私有原始报告到公开回放的一致性；其余单元、Mock、安全和公开实测数据计算测试不依赖原始日志。跳过不算通过。部分C++协议测试还需要本地编译器；缺失会明确SKIP。

发布版的`replay.public`从已公开的真实位姿/障碍采样重新计算速度、偏移、净距、反转、空间带积分，并核对可信计划和正式场景资产；它不伪装成对未发布原始模型/租约来源的独立认证。

`tests/replay_fixtures.py`只在测试临时目录生成明确标记的合成协议数据，用于Schema/身份/篡改/错误分支，不作实验成果。真实展示仍使用`web/replay/data/evidence.json`中的原有真实脱敏采样。`replay.build`保留维护者从私有原始记录完整重建的能力，原始数据缺失时失败，不静默伪造或回退。

本轮没有安装依赖、启动Gazebo、调用Qwen、发送导航目标或注入障碍。Vercel页面资源及真实回放JSON未修改。

## README和实验结论核对

README包含定位、实现/验证范围、架构与信任边界、四站流程、D1/D2/D3对照、四张指标图、安全/心跳/异常、离线安装测试、操作者仿真运行、现有Demo链接及Roadmap。

持续保留以下限制：D3安全净距保护提前停止，运动实验FAILED，未完成25秒；最终V4四站成功。Contact为NOT_MEASURED，不声称零碰撞；21次Path变化不等于正式Planner调用。空间偏移和减速是独立单次实验的描述性计算，不作因果结论。

## 远端验收方法与未完成事项

上传前读取GitHub原始REST元数据，确认`private=true`及`visibility=private`，并检查已有分支历史，不强制覆盖远端。通过已授权GitHub连接上传审计后的文件，不在shell注入GitHub令牌。

提交后读取远端Git tree，对每个路径、文件模式、Git blob哈希与本地副本逐项比较；读取远端README及图片对象，核对相对链接、PNG有效性和字节身份，再次确认仓库私有。最终提交SHA以GitHub历史和交付回执为准，文档不嵌入自身递归哈希。

尚未完成的产品/发布事项：整体开源许可证与第三方许可公开发布复核、Ollama真实测试、实机验证、重复动态交会统计、Contact/延迟测量。详细技术限制见README及[实验结果](STAGE62_RESULTS.md)。保持私有不代表这些未验证功能已经完成。
