# 文档同步到飞书的 CI 设计

## 1. 文档定位

这份文档描述的是：如何用 GitHub Actions 把仓库里的 `docs/` 同步到飞书。

当前目标先不是把 workflow 立即写死，而是先把方案边界、目录约束、凭据管理和同步策略说明清楚，避免后面把 CI 写成一次性脚本。

## 2. 目标

期望达到的效果：

- 当 `docs/` 发生变更时，自动触发同步
- 通用文档和场景文档在飞书里保持相似层级
- 尽量避免整库全量覆盖，优先做按文件增量同步
- 保留仓库作为源事实，飞书作为发布端

## 3. 推荐同步策略

建议把 GitHub 仓库视为唯一源头：

- GitHub `docs/` 是编辑源
- 飞书文档空间是发布目标

推荐不要在飞书中反向编辑后再同步回仓库，否则很容易出现双向冲突。

## 4. 飞书侧推荐结构

建议在飞书中也保持与仓库接近的目录结构，例如：

```text
World Model Docs
  General
    Architecture
    Repository Structure
    ROS2 Interfaces
    Safety and Validation
    Feishu Sync CI
  Scenarios
    Indoor
      README
      Architecture
      MVP Plan
      ArduPilot SLAM Design
```

这样做的好处是：

- 文档定位一致
- 仓库路径和飞书路径容易映射
- CI 更容易维护一份静态映射表

## 5. 推荐实现方式

### 方案 A：Markdown -> Feishu Doc 内容块

思路：

- GitHub Actions 检测变更的 Markdown 文件
- 用脚本读取 Markdown
- 转换成飞书文档块结构
- 调用飞书 OpenAPI 创建或更新文档

优点：

- 可控性强
- 方便做按文件同步
- 能稳定保持“一个 Markdown 文件对应一个飞书文档”

缺点：

- 需要自己做 Markdown 到飞书文档结构的转换
- 表格、Mermaid、复杂代码块等格式需要额外处理

### 方案 B：先转中间格式，再导入

思路：

- 先把 Markdown 转成飞书更容易接收的中间格式
- 再调用飞书导入或文档创建相关接口

这个方向是否值得采用，要看你后面确认的飞书接口能力和格式保真要求。

**我基于当前官方文档检索到的 API，更倾向先走方案 A。**

这是一个工程推断：因为我已经确认飞书官方有“获取 tenant access token”“创建文档”“创建 wiki 节点”这些 API，但我没有检索到一个明显、稳定、专门面向 Markdown 直传的官方入口，所以首版更稳妥的是自己控制转换和发布流程。

## 6. 推荐的 GitHub Actions 触发方式

建议首版 workflow 只在文档变更时触发，例如：

- `push` 到主分支
- 且变更路径命中 `docs/**`

可选再加一个手动触发：

- `workflow_dispatch`

这样做的好处是：

- 便于先小范围验证
- 出问题时可以手动重跑

## 7. 推荐的仓库内组件

建议不要把所有逻辑塞进 workflow YAML，而是拆成：

- `.github/workflows/docs-feishu-sync.yml`
- `scripts/feishu_sync.py`
- `configs/feishu_docs_map.yaml`

### `.github/workflows/docs-feishu-sync.yml`

职责：

- 触发 CI
- 安装依赖
- 注入 secrets
- 调用同步脚本

### `scripts/feishu_sync.py`

职责：

- 扫描变更文件
- 读取 Markdown
- 做格式转换
- 调用飞书 OpenAPI
- 输出同步结果

### `configs/feishu_docs_map.yaml`

职责：

- 维护本地文档路径与飞书目标节点的映射

例如：

```yaml
docs/general/architecture.md:
  section: general
  title: Architecture
docs/scenarios/indoor/mvp_plan.md:
  section: scenarios/indoor
  title: MVP Plan
```

## 8. 建议使用的 GitHub Secrets

根据 GitHub 官方文档，workflow 可以通过 `secrets` 上下文读取仓库 secrets，因此建议至少配置：

- `FEISHU_APP_ID`
- `FEISHU_APP_SECRET`
- `FEISHU_WIKI_SPACE_ID`
- `FEISHU_PARENT_NODE_TOKEN`

如果后面区分测试空间和正式空间，还可以进一步拆成 environment secrets。

## 9. 推荐同步流程

建议脚本按下面步骤执行：

1. 获取 GitHub Actions 里变更的文档列表
2. 根据映射表找到每个文件对应的飞书目标位置
3. 调用飞书认证接口获取 `tenant_access_token`
4. 确认飞书 wiki 节点是否存在，不存在则创建
5. 创建或更新对应文档
6. 把本地 Markdown 转成飞书可接受的结构
7. 写入内容
8. 输出同步报告

## 10. 文档格式处理建议

首版建议优先支持这些 Markdown 元素：

- 标题
- 段落
- 列表
- 链接
- 代码块
- 表格

对于这些内容，建议先定义降级策略：

- Mermaid：先转为代码块或跳过渲染
- 超复杂表格：必要时降级成普通文本
- 相对链接：转换成仓库绝对路径或飞书内部链接映射

## 11. 文档交叉引用如何处理

既然你已经明确希望文档之间相互引用，这个 CI 里最好也考虑链接策略。

建议分两层：

### 仓库内写法

文档里继续使用仓库路径，例如：

- `docs/general/architecture.md`
- `docs/scenarios/indoor/mvp_plan.md`

### 发布到飞书时

同步脚本可以读取映射表，把这些仓库路径链接替换成对应的飞书文档链接。

这样可以同时满足：

- 本地仓库阅读顺畅
- 飞书阅读时也能点跳转

## 12. 风险点

实现这个 CI 时，最容易踩坑的地方通常是：

- Markdown 和飞书文档格式不完全等价
- 相对链接在飞书里失效
- Mermaid 和代码块保真度不一致
- 飞书节点结构和仓库路径结构不一致
- 多次同步导致重复创建文档

所以建议从第一版起就坚持：

- 一份本地文件对应一个固定飞书节点
- 使用稳定映射表
- 先小范围验证，再扩大同步范围

## 13. 当前建议的落地顺序

建议分三步：

1. 先补齐文档交叉引用
2. 再写 `feishu_sync.py` 的本地脚本
3. 最后再加 `.github/workflows/docs-feishu-sync.yml`

这样调试成本最低，因为飞书 API 和格式转换问题可以先在本地跑通，再交给 CI。

## 14. 官方资料参考

- GitHub Actions secrets:
  - https://docs.github.com/en/actions/how-tos/write-workflows/choose-what-workflows-do/use-secrets
- GitHub Actions contexts:
  - https://docs.github.com/en/actions/reference/workflows-and-actions/contexts
- Feishu 获取 tenant access token:
  - https://open.feishu.cn/document/server-docs/authentication-management/access-token/tenant_access_token_internal
- Feishu 创建文档:
  - https://open.feishu.cn/document/server-docs/docs/docs/docx-v1/document/create
- Feishu 创建 wiki 节点:
  - https://open.feishu.cn/document/server-docs/docs/wiki-v2/space-node/create
