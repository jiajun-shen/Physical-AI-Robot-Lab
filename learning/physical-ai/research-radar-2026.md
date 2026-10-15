# 2026 Physical AI 研究雷达

> 更新：2026-10-15（Europe/Paris）。
>
> 这是一份精选学习清单，不是统一排行榜，也不是任何模型已经被接入
> `Physical-AI-Robot-Lab` 的声明。日期和能力以链接的一手来源为准。

## 阅读顺序

1. **开放 VLA 与数据/评测**：先理解输入、动作表示、数据格式与闭环评测。
2. **世界模型与仿真**：理解预测、合成数据和 sim-to-real 的边界。
3. **分层具身智能**：理解高层推理、低层控制、任务分解和执行验证如何分工。
4. **回到本项目**：把可学习理念转为可审计的真实运行实验，而不是直接复制模型名称。

## 研究条目

| 方向 | 工作与发布日期 | 一手来源 | 为什么值得学习 | 当前接入状态 |
|---|---|---|---|---|
| 开放 VLA | **OpenVLA**（2024-06-13） | [官方项目页](https://openvla.github.io/) | 提供“视觉 + 语言 + 机器人状态 → 连续动作”的开放 VLA 基线，适合作为理解跨机械臂预训练、参数高效适配和评测接口的起点。 | 学习参考；未接入。 |
| 通用机器人控制 | **π₀: A Vision-Language-Action Flow Model for General Robot Control**（2024-10-31） | [Physical Intelligence 官方研究页](https://www.pi.website/blog/pi0) | 学习 flow-matching 动作生成、跨具身数据混合与通用策略微调；关注其动作表示和闭环执行假设。 | 学习参考；未接入。 |
| 开放世界泛化 | **π₀.₅: a VLA with Open-World Generalization**（2025-04-22） | [官方研究页](https://www.pi.website/blog/pi05) | 把评测重心从训练场景内技能转到未见家庭环境、多步骤任务与泛化；直接启发本项目的随机场景、重新观察和再规划验收。 | 学习参考；未接入。 |
| 轻量本地 VLA | **SmolVLA**（2025-06-03） | [Hugging Face 官方发布](https://huggingface.co/blog/smolvla) | 450M 级开放 VLA、社区数据与异步推理路径，对未来消费级设备上的独立实验更现实。 | 候选研究环境；未接入 ROS 工作区。 |
| 世界模型 | **V-JEPA 2: Self-Supervised Video Models Enable Understanding, Prediction and Planning**（2025-06-11） | [Meta 论文页](https://ai.meta.com/research/publications/v-jepa-2-self-supervised-video-models-enable-understanding-prediction-and-planning/) | 结合视频自监督与少量机器人交互数据，研究感知、预测与规划；适合作为“预测—MPC—重规划”学习线索。 | 学习参考；未接入。 |
| 世界生成 | **Genie 3**（2025-08-05） | [Google DeepMind 官方发布](https://deepmind.google/blog/genie-3-a-new-frontier-for-world-models/) | 提供交互式世界模型的前沿参照，可用于思考生成式仿真、长时程一致性和数据扩展，但不等价于可用机器人物理引擎。 | 架构参考；未接入。 |
| 具身推理 + VLA | **Gemini Robotics 1.5 / Gemini Robotics-ER 1.5**（2025-09-25） | [Google DeepMind 官方发布](https://deepmind.google/blog/gemini-robotics-15-brings-ai-agents-into-the-physical-world/) | 高层 embodied reasoning 负责任务分解、工具调用、进度/成功判断，低层 VLA 执行动作；是未来 TaskGraph 与 observe/verify/replan 架构的重要参照。 | 架构参考；访问受限，未接入。 |
| 开放 humanoid 基础模型 | **NVIDIA Isaac GR00T N1.6**（2025-12-15） | [NVIDIA Research 项目页](https://research.nvidia.com/labs/gear/gr00t-n1_6/) | 覆盖多具身、真实/仿真数据、相对动作和长时程操作；适合学习训练数据混合、后训练与误差累积的工程权衡。 | 学习参考；未接入。 |
| 闭环经验学习 | **π*₀.₆: a VLA That Learns From Experience**（2025-11-17） | [Physical Intelligence 官方研究页](https://www.pi.website/blog/pistar06) | 强调 demonstration、offline RL 与机器人纠错/奖励反馈；说明可靠性来自“执行—观察—纠错”的闭环数据，而非单次离线策略。 | 学习参考；未接入。 |
| 机器人基础模型工程 | **NVIDIA Isaac GR00T 1.7**（2026-07-07） | [NVIDIA 官方技术发布](https://developer.nvidia.com/blog/develop-humanoid-robot-policies-end-to-end-with-nvidia-isaac-gr00t/) | 把仿真、遥操作、LeRobot 数据、后训练、评测和部署连成工程流程；对未来数据管线与闭环评测很有价值。 | 学习参考；重 GPU/CUDA 依赖，未接入。 |
| 最新分层 Physical AI | **Gemini Robotics 2 / ER 2 / On-Device 2**（2026-07-30） | [Google DeepMind 官方发布](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/)；[模型总览](https://deepmind.google/models/gemini-robotics/) | VLA 负责动作，ER 负责视觉/时空推理、长任务编排和成功判断；与“语言指令 → TaskGraph → 连续执行 → 每步重新观察/验证/重规划”的目标高度相关。 | 架构参考；服务/模型访问受限，未接入。 |
| 仿真长程基准 | **RoboCasa365**（ICLR 2026） | [官方项目页](https://robocasa.ai/) | 365 个家务任务、2500 个厨房环境，适合学习长时程任务组合、泛化与层级评测设计。 | 基准参考；未接入。 |
| 真实机器人数据 | **DROID**（2024-03-19 / RSS 2024） | [论文](https://arxiv.org/abs/2403.12945)；[官方数据页](https://droid-dataset.github.io/droid/) | 大规模、多场景、多任务真实操作数据；适合学习数据采集、跨场景泛化与可复现实验协议。 | 数据参考；未下载。 |

## 学习与工程边界

- 这些资料的“强结果”来自各自作者的硬件、数据、许可证和评测条件；不能外推成 ABB IRB1200 的能力。
- 现有本地 WSL 环境的显存/内存、CUDA Toolkit 和 Docker 条件不满足大型 VLA 的默认训练假设。未来实验必须在隔离环境中进行。
- 本项目后续所有视觉/策略能力需要保留安全层、硬件限制、真实反馈、失败处理和可复现证据。
- 研究阅读不取代基础工程：传感器标定、TF、碰撞检查、控制器限幅、数据质量和运行时监督仍是 Physical AI 系统的必要条件。

## 下一轮建议问题

1. 哪一类任务最适合作为 ABB 实验室的最小 VLA/视觉闭环试验？
2. 怎样定义不读取 Gazebo ground truth 的视觉评测协议？
3. 每条 VLA 动作如何经过现有 MoveIt/ros2_control 安全边界？
4. 如何记录演示、失败、恢复与成功判据，使结果可复查？
