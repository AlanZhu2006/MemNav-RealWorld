# Documentation Index

本页是仓库文档的统一索引。实验前先看当前状态，再根据任务进入操作、设计或组件文档。
日期型文档保留历史状态；当前入口是 CURRENT_STATUS.md。

## 当前事实与操作

| 文档 | 唯一职责 |
| --- | --- |
| [`CURRENT_STATUS.md`](../CURRENT_STATUS.md) | 当前实现、论文证据、延迟回放和工作区状态 |
| [`REALWORLD_MEMORY_STORAGE_SYNC_20260916_CN.md`](../REALWORLD_MEMORY_STORAGE_SYNC_20260916_CN.md) | 显式存储配置、Survey 兼容和最新冻结请求延迟 |
| [`LOCAL_STORAGE.md`](LOCAL_STORAGE.md) | 缓存清理、原始记录保留和本地恢复 |
| [`REALWORLD_EXECUTION_ADAPTATION_CHANGELOG_20260907.md`](../REALWORLD_EXECUTION_ADAPTATION_CHANGELOG_20260907.md) | 9 月 7 日执行适配的具体改动、默认与可选行为、纯旋转风险和部署步骤 |
| [`REALWORLD_RECEDING_HORIZON_AUDIT_20260907.md`](../REALWORLD_RECEDING_HORIZON_AUDIT_20260907.md) | 004/006/009 失败归因、执行适配与离线重判 |
| [`REALWORLD_LOCKED_LATENCY_RESULT_20260907.md`](../REALWORLD_LOCKED_LATENCY_RESULT_20260907.md) | 真实 Jetson/RTX 静止延迟及其验证边界 |
| [`REALWORLD_EXPERIMENT_HANDBOOK_CN.md`](../REALWORLD_EXPERIMENT_HANDBOOK_CN.md) | 完整中文现场实验与交接手册 |
| [`RUNBOOK.md`](../RUNBOOK.md) | Full-Mono 启动、检查、停止和故障注入速查 |
| [`TWO_PASS_REVISIT_RUNBOOK.md`](../TWO_PASS_REVISIT_RUNBOOK.md) | sealed Survey → Formal Revisit 两阶段流程 |
| [`EXPERIMENT_DATA_COLLECTION.md`](../EXPERIMENT_DATA_COLLECTION.md) | ROS bag、receipt、Foxglove 与第三视角证据封存 |
| [`REALWORLD_EVALUATION.md`](../REALWORLD_EVALUATION.md) | 历史 4 场景 × 5 paired blocks 协议，非当前论文分母 |

## 架构与组件

| 文档 | 唯一职责 |
| --- | --- |
| [`ARCHITECTURE.md`](../ARCHITECTURE.md) | 双机权限、协议状态机、失败语义与兼容策略 |
| [`deployment/go2/STACK_MODULES_CN.md`](../deployment/go2/STACK_MODULES_CN.md) | 启动脚本层级、navigation profile 与 arrival 组合 |
| [`deployment/go2/README_CN.md`](../deployment/go2/README_CN.md) | Jetson、D435i、Go2 和原生 baseline 组件级说明 |
| [`deployment/gpu/README.md`](../deployment/gpu/README.md) | RTX 服务配置和外部研究依赖 |
| [`deployment/odin1_gt/README_CN.md`](../deployment/odin1_gt/README_CN.md) | 独立 Odin reference/evaluation sidecar |

## 发布、来源与许可

| 文档 | 唯一职责 |
| --- | --- |
| [`SOURCE_MANIFEST.md`](../SOURCE_MANIFEST.md) | 源码来源、冻结 payload 和明确排除项 |
| [`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md) | 上游和第三方许可说明 |
| [`media/README.md`](../media/README.md) | 可公开 demo 媒体索引与哈希 |
| [`archive/`](archive/) | 已被取代但仍保留审计价值的日期型发布/联调记录 |

运行行为以当前代码和该次 resolved contract 为准；实验结果以绑定的原始记录为准。
CURRENT_STATUS.md 提供最新索引。历史 release manifest 和日期报告保留当时含义。
