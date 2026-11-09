# 计划语义ID与模型来源关联

## 根因（先只读复现）

对现有`plans/real_api/qwen_response.json`与`qwen_plan.json`的比较结果：内嵌可信计划对象完全相同。
`qwen_plan.json`恰好是`json.dumps(plan, ensure_ascii=False, indent=2)`的文本，没有结尾换行。

旧Agent对缩进JSON加一个换行后计算SHA-256，得到：

```text
acce4a922afc0b9fa6f2446f2e1e908ab790c72750097612a2b9cdf06c081ceb
```

旧execute_plan对所读文件文本计算SHA-256，得到：

```text
8b24495133f3cfe27c3de19dab7c3b4aba32fc5afeabbd64e7801f218190be9f
```

V4旧实现只照抄`--plan-id`。Planner V1只生成包含created_at的计划，没有生成另一个ID。
所以这两个字符串本来都意图标识同一份可信计划，差异来自序列化末尾换行，不是任务或来源层次不同。

## 标识与版本

| 字段 | 语义 |
| --- | --- |
| request_id | Provider返回的模型调用标识；没有时null，不自行伪造 |
| plan_id | 经过校验的执行计划的语义摘要；同一计划可有多个模型请求和多次执行 |
| run_id | 单次机器人执行的标识；正常入口每次新建UUID |
| provenance_id | 来源记录摘要；区分同一plan_id的不同模型请求和用量记录 |

新provenance版本2.0、新执行报告版本2.0；计划本身仍为1.0，避免向可信计划加入未声明字段。
plan_id必须是64位小写十六进制SHA-256，另带`plan_id_scheme=trusted-plan-sha256-v1`。

## 确定性算法

统一实现位于`planner/plan_identity.py`，Agent、保存校验、执行入口和V4共用。

1. 先校验严格计划Schema；非法动作、额外字段、非有限数值及不连续步骤均不能计算受支持的ID。
2. 取出算法版本、schema_version、scene_id、frame_id，以及有序steps中的step_id/action/location/x/y。
3. 坐标转换为确定性的十进制字符串，不按空间容差取整。等值整型/浮点型、正负零和指数写法归一。
4. 对键排序，以UTF-8、ensure_ascii=False、紧凑分隔符序列化该投影，再计算SHA-256。

不包含created_at、READY状态、文件名、路径、JSON排版、键排列、Provider、请求ID或执行参数。
READY仍是输入校验条件；排除状态字段不代表接受其他状态。
场景、坐标系或步骤改变会改变ID或导致Schema拒绝；顺序和重复绝不排序/去重。
现有1.0计划只支持NAVIGATE和map，其他动作/坐标系直接拒绝。

哈希本身不等于验证证明：执行路径还必须重新加载地点注册表，精确比较目标坐标，再计算/核对ID。
为防止“哈希标识坐标A、实际却发送略有不同坐标B”，已移除原0.001容差替换；偏移即拒绝。
这是有意收紧的校验规则。数字的等值表达（例如1与1.0）不受影响。

## 来源与执行边界

- Agent仅从ProviderResult的明确属性提取provider、model、requested_model、request_id、Token用量和duration_seconds，不合并模型JSON中的身份字段，也不调用可被覆盖的as_dict来填写保留字段。
- LLM意图Schema仍只允许动作和地点；plan_id、run_id或来源字段出现在模型意图中会被拒绝。
- 来源记录按类型检查并独立求摘要，绑定到语义plan_id。原始模型响应保留在脱敏provenance中，不提升为执行身份。
- Agent标准导出目录中的provenance.json由execute_plan/V4自动发现；非标准布局应显式提供`--provenance`。
- execute_plan传递完整计划路径、预期语义ID、来源路径和来源摘要。V4在导入ROS/创建节点前重新读取、校验，并在run_tasks进入执行前再验证。
- 计划实质修改、来源摘要变化、非法ID或ID与内容不符，均在发送目标前失败。
- 报告包含完整计划、plan_id与版本、run_id以及受限来源记录；上层核验全部身份、任务结果和恢复决策。
- 同一计划的来源文件是显式选择的，不能只凭plan_id推断是哪一次模型请求。

为兼容手动坐标工具，无`--plan`的V4仍可显式执行坐标，但报告标为coordinates-only，plan_id与source均为null。
该入口不能仅传plan_id/来源来假装已验证计划；需要可信追踪的Agent任务必须走完整计划路径。

## 历史兼容（不改写）

旧Agent哈希命名为`legacy-agent-pretty-json-sha256-v0`，旧文件文本哈希命名为`legacy-file-text-sha256-v0`。
历史JSON、过去的plan_id、request_id、run_id和状态保持原样。

旧provenance导入要求显式`--allow-legacy-provenance`，先校验旧哈希和原内嵌计划，再在内存建立新ID及legacy_reference映射。
新正常执行报告验证不接受旧报告冒充2.0结果。只读审计工具可按原文件哈希及有序目标关联旧1.0报告，不会补写历史来源信息。

## 本次真实留存文件验收（未重放导航）

两份现有计划通过完整校验，统一语义ID为：

```text
e205afb6f4eca79a73bb9a9d0513c64d156df61a1ab5b9e053fab869f9e738e3
```

来源保留：provider=qwen、model=qwen-plus、request_id=`chatcmpl-152b76af-0d80-9c2e-a04f-2b854f0c82c7`，输入/输出/总Token=219/26/245，模型请求耗时1.660603秒。
现有报告`logs/task_v4_887222def3fd4c908583963a130c0766.json`中的run_id为`887222def3fd4c908583963a130c0766`，整体SUCCEEDED。
旧Agent ID和文件ID如上保留，通过旧哈希、计划内容和有序坐标验证关联到新语义ID。
这是一份事后只读关联结果，不表示历史报告当时就记录了模型来源或新版ID。

以下均为只读/Dry Run，不发送导航目标：

```bash
cd ~/embodied-agent
python3 -B -m planner.trace_audit \
  --response plans/real_api/qwen_response.json \
  --plan plans/real_api/qwen_plan.json \
  --report logs/task_v4_887222def3fd4c908583963a130c0766.json

python3 -B -m planner.execute_plan \
  --plan plans/real_api/qwen_plan.json \
  --provenance plans/real_api/qwen_response.json --allow-legacy-provenance

python3 -B -m day4.task_executor_v4 \
  --plan plans/real_api/qwen_plan.json \
  --provenance plans/real_api/qwen_response.json --allow-legacy-provenance

PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider
```

## 测试与限制

460项全部通过：原405个用例保留且通过，新增55项；未安装依赖或调用真实API。
原测试中旧字节哈希断言、容差替换断言、可信执行模拟桥接已更新为新契约，没有删除、跳过或放宽这些行为检查。
新增历史留存文件测试在本WSL运行通过；全新checkout若没有该用户留存文件，该项会明确skip，其他测试不依赖历史私有数据。

修改前快照：`.snapshots/plan_identity_20261108T102422Z/`。历史plans/logs/configs及Action状态机、Recovery Policy未修改。

限制：

- SHA-256是内容/关联完整性标识，不是数字签名；拥有本地文件写权限的人仍可同时重算摘要，不能据此认证历史作者。
- 地图/场景物理资产未在1.0计划中版本化；相同scene_id下更换地图不会自动进入摘要，部署应另行维护场景版本。
- 旧报告没有模型来源字段，事后只能说明留存文件哈希及坐标一致，不能伪造当时缺失的采集证据。
- 未提供来源文件的独立计划，其source为null，不从文件名或plan_id猜测模型来源。
- 修改后的真实ROS2回归尚未执行，仍需用户另行显式触发。
