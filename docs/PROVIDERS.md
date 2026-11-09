# 第三阶段：Rule / Qwen / Ollama 统一规划接口

本阶段只做离线Mock测试，未调用真实模型、启动仿真或发送导航目标。
第二阶段的Action状态管理、Recovery Policy、退出码、原子执行报告和严格计划校验保持不变。

## 接口与信任边界

`create_provider(env=..., transport=...)`显式选择Provider；省略env时才读取进程环境。
统一方法：`generate_intent(instruction, scene) -> ProviderResult`。

结果字段：

| 字段 | 含义 |
| --- | --- |
| `content` | 不可信模型文本，必须经过本地校验 |
| `raw_response` | 原始响应对象，已脱敏本次Qwen密钥；不含请求头 |
| `provider` | rule / qwen / ollama |
| `model` | 服务返回的模型名；缺失时记录请求模型 |
| `requested_model` | 实际请求的配置模型名，区分服务端别名解析 |
| `request_id` | 响应id；没有时使用x-request-id或x-dashscope-request-id；均无则null |
| `input_tokens` / `output_tokens` / `total_tokens` | 分别对应usage字段；缺失保留null，不自行计算或填0 |
| `duration_seconds` | 请求耗时；Rule则为规则解析耗时 |

仅`plan_instruction()`负责接收Provider结果并执行：

```text
校验指令与场景
 → 本地显式导航契约：解析全部步骤、检查未知地点，无法完整解析则停止
 → Provider（只发送指令、合法地点名，不发送注册表坐标）
 → 严格JSON解码和意图Schema
 → NAVIGATE白名单、地点注册表、1—20步限制
 → 与输入契约逐项比较：地点、顺序、重复次数必须完全一致
 → 复用build_plan生成可信坐标
 → 第二阶段的计划Schema校验
 → 返回计划和来源记录
```

意图只允许`steps`，每步只允许`action`和`location`。坐标、ROS命令、工具调用、执行器参数及其他额外字段一律拒绝。
不从Markdown代码块提取JSON，不修补非法输出，不接受被截断或未确认完成的响应，不自动尝试第二次模型请求。
形式合法不能证明模型正确理解指令；新入口没有执行功能，仍需独立审阅和执行前校验。

## 配置

`.env.example`只含空密钥占位和示例，不自动加载。将实际选择的变量导出到当前shell。
只读取已选Provider的配置；Rule不读取Qwen密钥，Ollama不携带Qwen认证头。

| 变量 | 默认/要求 |
| --- | --- |
| `LLM_PROVIDER` | `rule`；只接受rule / qwen / ollama |
| `LLM_TIMEOUT_SECONDS` | 30；正有限数，最大300；网络I/O超时，不是严格的整个运行墙钟期限 |
| `QWEN_MODEL` | `qwen-plus` |
| `QWEN_BASE_URL` | `https://dashscope.aliyuncs.com/compatible-mode/v1` |
| `QWEN_API_KEY` | Qwen必需，只从环境读取 |
| `OLLAMA_BASE_URL` | `http://localhost:11434/v1` |
| `OLLAMA_MODEL` | 必须填写已安装的本地模型，没有默认假设 |

Qwen Base URL可按地域和工作空间配置，要求HTTPS、无内嵌账号密码/查询参数/fragment。
阿里云官方文档说明地域端点和密钥需要匹配；此处保留用户指定的默认端点，不自行迁移地域。

Ollama允许localhost、回环地址和显式配置的私有IP（用于WSL宿主连接），拒绝公共云端点及已知`:cloud`/`-cloud`模型标签。
不会探测、下载或安装模型；未安装模型由服务返回错误。不能从兼容API证明一个自定义别名一定指向本地模型；应在Ollama服务端关闭云功能并核实本地模型配置。

## 切换示例

下面只展示配置方式。本阶段实际只运行Rule与Mock测试；Qwen命令会在后续获准的真实API阶段使用。

```bash
# 默认纯规则；不联网，不发送导航目标
LLM_PROVIDER=rule python3 -B -m planner.agent_planner '先去A区，再去B区'

# 显式保存到一个全新的目录；不覆盖历史计划/日志
LLM_PROVIDER=rule python3 -B -m planner.agent_planner '去A区' \
  --output-dir /tmp/embodied-agent-new-plan

# Qwen配置：需另行安全设置QWEN_API_KEY；本阶段不运行模型请求
export LLM_PROVIDER=qwen
export QWEN_MODEL=qwen-plus
export QWEN_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1

# Ollama配置：替换为实际已安装的本地模型名；本阶段不连接服务
export LLM_PROVIDER=ollama
export OLLAMA_BASE_URL=http://localhost:11434/v1
export OLLAMA_MODEL='替换为本地已安装模型名'
```

切换后仍使用同一个`python3 -B -m planner.agent_planner '指令'`入口；直接脚本方式也兼容。
选择qwen会产生潜在计费请求，选择ollama会连接所配置服务；默认rule且无静默切换。
`.env`文件不会被程序自动加载，密钥不要写入命令历史或提交到Git。

## 输出与后续执行

默认只向stdout输出可信坐标计划和来源摘要，不写文件，不打印原始模型文本。
显式`--output-dir`才创建新目录，先原子写入脱敏`provenance.json`，再写入`plan.json`；已有目录拒绝覆盖，失败尽可能清理本次不完整输出。

来源文件记录完整脱敏响应、模型和用量、原指令SHA-256、场景SHA-256及版本化计划语义SHA-256。
计划本身保持原1.0格式。该语义ID通过新版执行链路与V4报告关联；历史文件哈希须按旧版本显式映射，详见[计划标识规范](PLAN_IDENTITY.md)。

下面的后续命令仍只做预检查：

```bash
python3 -B -m planner.execute_plan --plan /tmp/embodied-agent-new-plan/plan.json
```

Agent入口没有`--execute`选项，不导入ROS、不调用Action或启动子进程。真实导航仍由用户单独显式触发。

## 错误与计费边界

- 缺密钥：`MISSING_API_KEY`；Ollama未配置模型：`MISSING_OLLAMA_MODEL`。
- 401：`AUTHENTICATION_ERROR`；403：`PERMISSION_DENIED`。
- 429：`RATE_LIMITED`；已知额度/欠费错误码：`QUOTA_EXHAUSTED`（优先于HTTP通用分类）。
- 网络错误：`NETWORK_ERROR`；超时：`REQUEST_TIMEOUT`；5xx：`SERVER_ERROR`。
- 其余非200响应：`HTTP_ERROR`，包括不存在模型等404。
- 非法JSON：`INVALID_JSON`；非法意图结构：`INVALID_INTENT_SCHEMA`。
- 非法动作：`ILLEGAL_ACTION`；未知地点：`UNKNOWN_LOCATION`。
- 响应截断/未完成、工具调用等也会停止规划。

只传播固定错误码和HTTP状态，不传播服务端原始报错文本或底层异常字符串。
HTTP传输没有自动重试、重定向或环境代理，不会把Authorization转发给重定向目的地。
没有自动重试配置；如失败后需要再次请求，必须由用户显式重新发起，可能再次计费。
SDK并非必需：使用Python标准库HTTP，不新增运行依赖，也没有安装依赖。
每次请求最多2048输出token；输入长度、响应字节数和意图文本长度另有上限。

## 离线测试

```bash
cd ~/embodied-agent
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider
```

测试保留网络禁用夹具，所有凭据都是测试中的人工字符串，所有HTTP请求均为Mock。
当前尚未验证真实Qwen账户/地域、模型可用性、Ollama已安装模型以及修改后的真实导航。

官方接口资料：

- [阿里云百炼OpenAI兼容接口](https://www.alibabacloud.com/help/en/model-studio/compatibility-of-openai-with-dashscope)
- [Ollama OpenAI兼容接口](https://docs.ollama.com/api/openai-compatibility)

## 本阶段交付验证

最终运行结果：**300项，300通过，0失败**；其中原155项全部通过，新增145项。
原155项测试、第二阶段源码、历史计划/日志/场景配置逐文件SHA-256核对均未改变。

本阶段修改`README.md`；新增`.env.example`、本说明、`planner/agent_planner.py`、
`planner/intent_validator.py`、`planner/schemas/task_intent.schema.json`、
`planner/providers/`下9个模块，以及`tests/test_providers.py`、
`tests/test_provider_transport.py`、`tests/test_agent_planner.py`。

`pyproject.toml`依赖声明无需变更：仍使用已有jsonschema/pytest，HTTP只使用Python标准库。
修改前快照及核验记录位于`.snapshots/providers_20261108T075746Z/`。
当前`.git`仍被只读空挂载遮蔽，没有修改Git信息或创建提交。

本阶段到此停止。真实Qwen API测试尚未进行，须用户另行确认后开始。

## 后续忠实性修复

第三阶段的300项是当时的基线；后续增加105项忠实性测试，当前总计405项全部通过。新Agent只接受可完整解析的显式顺序导航指令，不再把模型返回的合法子集直接标记为READY。详见[任务忠实性边界与回归](TASK_FIDELITY.md)。

后续已统一plan_id并新增来源关联，当前测试460项全部通过；原始第三阶段计数保留为阶段记录。详细规则见[PLAN_IDENTITY.md](PLAN_IDENTITY.md)。
