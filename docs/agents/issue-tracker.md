# Issue 追踪器：本地 Markdown

本仓库的 Issue 与规格以 Markdown 文件形式存放在 `.scratch/` 中。

## 目录约定

- 每项功能使用一个目录：`.scratch/<功能标识>/`
- 功能规格文件：`.scratch/<功能标识>/spec.md`
- 实现票据目录：`.scratch/<功能标识>/issues/`
- 每张票据一个文件：`.scratch/<功能标识>/issues/<编号>-<票据标识>.md`
- 编号从 `01` 开始，不得把多张票据合并为一个文件
- 票据状态记录在文件顶部附近的 `Status:` 行
- 阻塞关系记录在 `Blocked by: <编号>, <编号>` 行
- 讨论记录追加在文件末尾的 `## Comments` 标题下

路径中的 `<功能标识>` 与 `<票据标识>` 使用简短、稳定的英文 kebab-case 名称，正文使用中文。

## 发布与读取

当技能要求“发布到 Issue 追踪器”时，在 `.scratch/<功能标识>/` 下创建对应文件；目录不存在时一并创建。

当技能要求“获取相关票据”时，直接读取用户指定路径或编号对应的 Markdown 文件。

## Wayfinder 操作约定

`/wayfinder` 使用一份地图文件和多份子票据：

- 地图：`.scratch/<工作标识>/map.md`，正文包含当前记录、已有决策与仍待澄清的问题
- 子票据：`.scratch/<工作标识>/issues/<编号>-<票据标识>.md`
- 类型：通过 `Type:` 行记录 `research`、`prototype`、`grilling` 或 `task`
- 状态：通过 `Status:` 行记录 `claimed` 或 `resolved`
- 阻塞：通过 `Blocked by:` 行列出前置票据编号；所有前置票据均为 `resolved` 后，本票据才解除阻塞
- 工作前认领：先将 `Status:` 设为 `claimed` 并保存
- 完成后解决：在 `## Answer` 下追加结论，将状态改为 `resolved`，再把结论摘要与文件链接追加到 `map.md` 的已有决策部分

扫描待办时，只选择尚未解决、未被阻塞且未被认领的票据；若有多张，优先选择编号最小者。
