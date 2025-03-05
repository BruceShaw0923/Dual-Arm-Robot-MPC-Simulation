# -- coding: utf-8 --
"""
绘制 3D 双臂机器人模型示意图

假设每个臂为两连杆模型：
┌–––––┐ 连杆1长度 L1
│ 基座│————► 关节1
└–––––┘ │ 连杆2长度 L2
▼
末端执行器

采用简化的球面坐标来计算每条连杆的末端位置：
对于连杆1，其末端位置：
= 基座 + L1 * [ sin(phi1)*cos(theta1),
sin(phi1)*sin(theta1),
cos(phi1) ]
对于连杆2，累加第一连杆的末端位置，方向由 (theta1+theta2) 和 (phi1+phi2) 给出。

左右臂基座位置不同（此处设左臂在 (-0.2, 0, 0)，右臂在 (0.2, 0, 0)），并且角度分别设置成对称或镜像形式。
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # 导入 3D 绘图模块
plt.rcParams["font.sans-serif"] = ["SimHei"] # 设置中文字体为SimHei
plt.rcParams["axes.unicode_minus"] = False # 解决负号显示问题

# 参数设置
L1 = 1.0 # 第一连杆长度
L2 = 0.8 # 第二连杆长度

# 左臂基座及关节角（单位为弧度）
base_left = np.array([-0.2, 0, 0])
theta1_left = np.deg2rad(45) # 第一关节水平方向旋转
phi1_left = np.deg2rad(60) # 第一关节与竖直方向的夹角（球面坐标中 phi）
theta2_left = np.deg2rad(10) # 第二关节相对旋转（水平）
phi2_left = np.deg2rad(20) # 第二关节相对旋转（垂直）

# 右臂基座及角度（这里采用镜像设置）
base_right = np.array([0.2, 0, 0])
theta1_right = np.deg2rad(-45)
phi1_right = np.deg2rad(60)
theta2_right = np.deg2rad(-10)
phi2_right = np.deg2rad(20)

# 计算左臂关节位置（使用球面坐标计算）
# 左臂连杆1末端（关节1位置）：
joint1_left = base_left + L1 * np.array([
np.sin(phi1_left) * np.cos(theta1_left),
np.sin(phi1_left) * np.sin(theta1_left),
np.cos(phi1_left)
])

# 左臂末端执行器位置：在关节1基础上，再加连杆2
# 累计角度（简单叠加，适用于视觉示意，不一定严格符合机器人运动学理论）
theta_left_total = theta1_left + theta2_left
phi_left_total = phi1_left + phi2_left
end_effector_left = joint1_left + L2 * np.array([
np.sin(phi_left_total) * np.cos(theta_left_total),
np.sin(phi_left_total) * np.sin(theta_left_total),
np.cos(phi_left_total)
])

# 计算右臂关节位置
joint1_right = base_right + L1 * np.array([
np.sin(phi1_right) * np.cos(theta1_right),
np.sin(phi1_right) * np.sin(theta1_right),
np.cos(phi1_right)
])
theta_right_total = theta1_right + theta2_right
phi_right_total = phi1_right + phi2_right
end_effector_right = joint1_right + L2 * np.array([
np.sin(phi_right_total) * np.cos(theta_right_total),
np.sin(phi_right_total) * np.sin(theta_right_total),
np.cos(phi_right_total)
])

# 3D 绘图
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 绘制左臂（蓝色线条带圆圈标记）
ax.plot([base_left[0], joint1_left[0], end_effector_left[0]],
[base_left[1], joint1_left[1], end_effector_left[1]],
[base_left[2], joint1_left[2], end_effector_left[2]],
'bo-', linewidth=3, markersize=8, label='左臂')

# 绘制右臂（红色线条）
ax.plot([base_right[0], joint1_right[0], end_effector_right[0]],
[base_right[1], joint1_right[1], end_effector_right[1]],
[base_right[2], joint1_right[2], end_effector_right[2]],
'ro-', linewidth=3, markersize=8, label='右臂')

# 绘制基座（黑色正方形）
ax.scatter(base_left[0], base_left[1], base_left[2], c='k', marker='s', s=100)
ax.scatter(base_right[0], base_right[1], base_right[2], c='k', marker='s', s=100)

ax.set_title("3D 双臂机器人模型示意图")
ax.set_xlabel("X 轴")
ax.set_ylabel("Y 轴")
ax.set_zlabel("Z 轴")
ax.legend()
ax.grid(True)

# 设置轴比例为 1:1:1 （等比例缩放）
ax.set_box_aspect([1, 1, 1])

# 保存为 eps 图
plt.savefig("dual_arm_robot_3d.eps", format="eps")
plt.show()