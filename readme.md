# Dual-Arm Robot MPC Simulation

本项目利用 Python3 实现了基于模型预测控制（MPC）的双臂机器人协同控制仿真。项目代码包括机器人状态空间模型构建、MPC 控制器设计、仿真实验及仿真结果的可视化。下文说明如何复现源码及生成各项图形结果。

作者： BruceShaw0923 （秋风）

## 声明

### 开源仅供学习交流与参考，也仅仅是一次小尝试，禁止不加修改的抄袭，禁止未经通知的转载

### Open source is only for learning, communication, and reference, and it is merely a small attempt. Direct copying without modification is prohibited, as is reposting without prior notification.

## 目录结构

```
.
├── draw_dualarm.py         # 3D 机器人示意图绘制
├── mpc_dualarm_2.py             # MPC 仿真及绘图代码
├── README.md                    # 本文件
```

## 环境要求

- Python 3.10.8（或相近版本）
- [numpy 1.26.4](https://numpy.org/)
- [cvxpy 1.6.2](https://www.cvxpy.org/)
- [matplotlib 3.7.3](https://matplotlib.org/)

推荐使用 Anaconda 环境管理工具。

## 安装依赖

可通过 pip 安装依赖（建议在虚拟环境或 conda 环境中操作）：

```bash
pip install numpy==1.26.4 cvxpy==1.6.2 matplotlib==3.7.3
```

或使用 conda（如果已在 conda 环境中）：

```bash
conda install numpy==1.26.4 matplotlib==3.7.3
pip install cvxpy==1.6.2
```

## 运行说明

1. **3D 双臂机器人模型示意图**

   文件 `draw_dualarm.py` 提供了一个使用 matplotlib 3D 绘图工具绘制双臂机器人示意图的例子。该代码采用简化的球面坐标正向运动学计算各连杆末端位置，将左右臂以不同颜色展示，并将图形保存为 SVG 文件。

   运行方式：

   ```bash
   python draw_dualarm.py
   ```

   运行后，会弹出 3D 模型图窗口，并在目录下生成文件 `dual_arm_robot_3d.eps`。<img src="https://github.com/user-attachments/assets/0f34115c-4efa-4be1-96ab-1182e46ed19d" alt="dual_arm_robot_3d" width="500px" height="300px">.

3. **MPC 双臂机器人仿真**

   文件 `mpc_dualarm_2.py` 为 MPC 控制仿真代码。在该代码中：

   - 构建了基于离散状态空间模型的双臂机器人系统。
   - 采用滚动时域 MPC 方法生成最优控制，其中考虑了物理约束及协同约束（例如左右臂第1关节位置之差）。
   - 仿真过程中生成各关节及控制输入的轨迹图和协同约束验证图，并保存为 EPS 矢量图文件。

   运行方式：

   ```bash
   python mpc_dualarm_2.py
   ```

   运行后，程序会生成如 `mpc_dualarm_xd.eps`（各关节轨迹图）`mpc_dualarm_u.eps`（输入力矩图）与 `collab_constraint.eps`（协同约束验证图）等输出文件，并在屏幕上显示图形。

## 复现步骤

1. 配置 Python 环境并安装好依赖包；
2. 使用 `git clone https://github.com/BruceShaw0923/Dual-Arm-Robot-MPC-Simulation.git`命令拉取所有源码文件至同一目录；
3. 通过运行 `draw_dualarm.py` 获得机器人 3D 示意图；
4. 根据需要运行 `mpc_dualarm_2.py`观察 MPC 仿真结果；
5. 查看输出的 EPS 文件，确保所有图形与论文中展示结果一致。

## 注意事项

- 若在绘图过程中中文显示乱码，请确保在代码中已设置中文字体，例如：
  ```python
  plt.rcParams["font.sans-serif"] = ["SimHei"]
  plt.rcParams["axes.unicode_minus"] = False
  ```
- 如需调整 MPC 参数、参考轨迹等，可直接修改代码中相关变量及 get_x_ref() 函数。
- 若使用不同后端（例如 TkAgg、Qt5Agg 或 Agg）时遇到问题，请根据具体错误适当切换后端。

## 联系方式

如有问题或建议，或获取本人课程论文原文供学习 请联系：

Wechat:13815012242

mail:

    446381508@qq.com (实时通知)
    shaoqiufeng@shu.edu.cn (每周看一次)

---
