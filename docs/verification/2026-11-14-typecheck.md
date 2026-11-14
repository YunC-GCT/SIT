# 全项目类型检查与修复（2026-11-14）

本轮处理 Pylance 同类类型诊断，未修改采样算法的参数、搜索策略或实验结果统计。保留此前未提交的模块审计及文件清理改动。

## 检查结果

- Pyright 1.1.414，覆盖 86 个 Python 文件：初次检查 171 个错误；最终 0 错误、0 警告。
- Python 目标版本设为 3.10，与实车 ROS2 Humble 容器一致；实际回归在 Windows 本地虚拟环境 Python 3.12 执行。
- 完整 unittest：153 项通过（9.033 秒）。
- Pyflakes：通过；`git diff --check`：通过。

## 修复内容

- 为调度器的命令与交互 Mixin 声明所需属性，补齐可空起终点、配置路径、预览路径的检查。
- 不可信坐标校验入口接受原始输入，再在运行时校验；保留浮点坐标、空路径等非法输入测试，不通过关闭诊断隐藏错误。
- 平滑曲线与 JSON 路径载荷使用可兼容整数、浮点数的序列注解；保留原来的运行时校验。
- 初始化异常分支可能使用的变量，避免未赋值引用；收窄 RRT* 首解迭代值和图像读取结果。
- 为本地缺少 ROS2 包的兼容层声明实际使用的 Node/Twist 接口，执行器用结构化接口兼容离线模拟节点。rclpy 的动态第三方接口仍使用 Any，本地检查不能验证真实 ROS2 SDK 的全部方法签名。
- 测试替身使用明确的类型边界、继承或 mock 注入；OpenCV 透视变换保留 float32 数组。
- 增加 `pyrightconfig.json`，开发依赖固定 Pyright 版本，便于复现。

## 复现

在项目根目录、激活项目虚拟环境后执行：

```powershell
$env:PYTHONUTF8='1'
python -m pip install -r requirements-dev.txt
python -m pyright
python -m pyflakes core map_pipeline motion_control path_planner path_postprocess tests tools main.py
python -m unittest discover -s tests -v
```

Pyright 与 Pylance 共用类型检查基础，但本次结果来自命令行 Pyright；未直接读取 VS Code 编辑器诊断。未进行真机测试，未重跑 180 项地图性能实验，不更新原实验成功率。
