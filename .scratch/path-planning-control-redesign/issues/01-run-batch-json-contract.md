# 01 — 运行批次与统一 JSON 契约基础

**What to build:** 建立一次实验运行批次的生命周期和统一 JSON 结果契约，使任意模块都能独立报告输入、输出、参数、状态、日志和结构化错误，下游可以安全判断结果是否完整。

**Blocked by:** None — can start immediately

**Status:** resolved

- [x] 创建可生成唯一 `run_id` 的运行批次，并保存模块状态、时间和版本信息
- [x] 定义统一成功、失败、进行中和待执行状态及结构化错误字段
- [x] 定义地图、路径、路径段、指标和模块清单的 JSON 表达规则，包含单位、坐标含义、维度和数据类型
- [x] 提供模块 API/CLI 可复用的契约读写与校验能力
- [x] 校验失败结果不会覆盖已有成功结果，下游拒绝无版本或非成功结果
- [x] 用最小示例模块完成一次成功和一次失败运行，并保存日志与结果

## Answer

已在 `core/` 建立统一 JSON 契约与运行批次基础：`RunBatch` 管理运行目录、manifest、模块状态和日志路径；`make_result()`、`make_error()`、`write_result()`、`read_result()`、`require_success()` 管理统一结果读写与下游成功校验；地图、路径、路径段、指标和模块清单通过专门 payload 构造器表达单位、坐标、维度和数据类型。

已修复失败结果覆盖成功结果的保护逻辑，并让 manifest 拒绝已成功阶段回退到失败状态。新增 `python -m core.contracts validate` 契约校验 CLI、`tools/minimal_contract_module.py` 最小成功/失败示例模块，以及 `tests/test_core_contracts.py` 覆盖核心契约行为。
