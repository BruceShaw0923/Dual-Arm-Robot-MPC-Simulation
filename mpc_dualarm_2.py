# -- coding: utf-8 --
"""
MPC控制双臂机器人协同控制

依赖库：
numpy, cvxpy, matplotlib
安装方式（例如在命令行使用pip）：
pip install numpy cvxpy matplotlib
"""

import numpy as np
import cvxpy as cp
import matplotlib
import math

# 指定后端（如TkAgg）以避免 backend_qt 中的错误
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
plt.rcParams["font.sans-serif"] = ["SimHei"] # 设置中文字体为SimHei
plt.rcParams["axes.unicode_minus"] = False # 解决负号显示问题
#============================

# 1. 系统与仿真参数设置
#============================
dt = 0.1 # 采样时间
num_joints = 4 # 4个关节（左臂2个关节，右臂2个关节）
n = 2 * num_joints # 状态维数：前num_joints为关节位置，后num_joints为关节速度
m = num_joints # 控制输入维数

Tsim = 50 # 总仿真步数
N = 10 # 预测时域长度

# 构造离散时间状态空间模型（所有关节均采用相同的二阶积分模型）
I4 = np.eye(num_joints)
A_upper = np.hstack([I4, dt * I4])
A_lower = np.hstack([np.zeros((num_joints, num_joints)), I4])
A = np.vstack([A_upper, A_lower])
B_upper = 0.5 * (dt**2) * I4
B_lower = dt * I4
B = np.vstack([B_upper, B_lower])

# 状态、输入约束
# 位置（前4维）限制：[-pi, pi]
q_min = -np.pi * np.ones(num_joints)
q_max = np.pi * np.ones(num_joints)

# 速度（后4维）限制：[-5, 5]
v_min = -5.0 * np.ones(num_joints)
v_max = 5.0 * np.ones(num_joints)

# 合并状态约束
x_min = np.concatenate((q_min, v_min))
x_max = np.concatenate((q_max, v_max))

# 输入限制
u_min = -10.0 * np.ones(m)
u_max = 10.0 * np.ones(m)

# 协同任务约束参数：
# 要求左臂第一个关节与右臂第一个关节位置之差接近δ，允许误差ε
delta = 0.0 # 期望差值（可以根据任务设置期望相对位置）
epsilon = 0.1 # 允许误差

# MPC目标函数权重
Q = np.diag(np.concatenate((10 * np.ones(num_joints), 1 * np.ones(num_joints)))) # 对状态的权重
R = 0.1 * np.eye(m) # 对控制输入的权重
P = Q # 终端状态权重

# 参考状态轨迹（设定希望所有关节达到某个固定角度，速度为0）
# 定义一个参考状态函数，输入当前时间 t_now，返回参考状态 x_ref
# x_ref 的前 num_joints 部分是各关节参考位置，后面部分设为零（参考速度为0）
def get_x_ref(t_now):
    # 参数选择：角度单位弧度，ω 为角频率
    omega = 0.1
    q1 = 0.5 + 0.2 * math.sin(omega * t_now)
    q2 = 0.3 # 固定参考值
    q3 = 0.5 + 0.2 * math.cos(omega * t_now)
    q4 = 0.4 # 固定参考值
    q_ref = np.array([q1, q2, q3, q4])
    v_ref = np.zeros(num_joints) # 速度目标均为0
    return np.concatenate((q_ref, v_ref))


#============================

# 2. MPC 求解与仿真
#============================

# 初始化状态
x_current = np.zeros(n) # 初始状态全部置0
x_current[0: num_joints] = -0.5 # 可设置初始位置较参考值有偏差

# 记录结果，用于之后绘图
x_hist = [x_current.copy()]
u_hist = []
time_hist = [0]

# 为热启动保存上次求解的变量（设定为None初始）
U_prev = None

# 开始仿真（每步求解一次MPC问题）
for t in range(Tsim):
    t_now = t * dt # 当前时刻
    # 定义优化决策变量：状态变量与控制序列
    X = cp.Variable((n, N+1))
    U = cp.Variable((m, N))


    cost = 0
    constraints = []

    # 初始状态约束
    constraints += [X[:,0] == x_current]

    # 对预测时域内每一步 k，构造参考状态 x_ref_k，并构造目标函数与约束
    for k in range(N):
        # 动力学约束：X[:,k+1] = A*X[:,k] + B*U[:,k]
        constraints += [X[:, k+1] == A @ X[:, k] + B @ U[:, k]]
        # 状态约束： x_min <= X[:,k+1] <= x_max
        constraints += [X[:, k+1] >= x_min, X[:, k+1] <= x_max]
        # 控制输入约束： u_min <= U[:,k] <= u_max
        constraints += [U[:, k] >= u_min, U[:, k] <= u_max]
        
        # 协同约束示例：
        # 对第k+1步预测状态，保证左臂第一个关节与右臂第一个关节位置差接近delta
        # X[0,k+1]: 左臂第一个关节位置； X[2,k+1]: 右臂第一个关节位置
        constraints += [X[0, k+1] - X[2, k+1] >= delta - epsilon]
        constraints += [X[0, k+1] - X[2, k+1] <= delta + epsilon]
        
        # 当前时刻在预测时域第k步：时间 = t_now + k*dt
        x_ref_k = get_x_ref(t_now + k * dt)  
        # 积累目标函数：每步状态误差与控制输入开销
        cost += cp.quad_form(X[:, k] - x_ref_k, Q) + cp.quad_form(U[:, k], R)

    # 末端状态代价：参考时间为 t_now + N*dt
    x_ref_terminal = get_x_ref(t_now + N * dt)
    # 末端目标函数也计入终端状态误差
    cost += cp.quad_form(X[:, N] - x_ref_terminal, P)

    # 定义并求解优化问题
    prob = cp.Problem(cp.Minimize(cost), constraints)

    # 热启动：如果有上次求解结果，把它调整为当前预测时域的尺寸(补最后一列)
    if U_prev is not None:
        U_init = np.hstack([U_prev, np.zeros((m, 1))])
        U.value = U_init

    prob.solve(solver=cp.OSQP, warm_start=True)

    if prob.status != cp.OPTIMAL and prob.status != cp.OPTIMAL_INACCURATE:
        print("警告：在时刻 t=%d 求解器未能获得最优解，状态可能不稳定！" % t)
        break

    # 取第一个控制输入
    u_current = U[:, 0].value
    if u_current is None:
        print("警告：求解器未返回有效解！")
        break

    # 更新状态（仿真：用相同的离散动态模型更新）
    x_current = A.dot(x_current) + B.dot(u_current)

    # 将当前最优控制序列作为热启动用于下一时刻
    U_prev = U.value[:,1:] if N > 1 else None

    # 保存当前状态与控制
    x_hist.append(x_current.copy())
    u_hist.append(u_current.copy())
    time_hist.append((t+1) * dt)
#============================

# 3. 仿真结果绘图
#============================
x_hist = np.array(x_hist) # (Tsim+1, n)
u_hist = np.array(u_hist) # (Tsim, m)
time_hist = np.array(time_hist)

plt.figure(figsize=(12, 8))

# 绘制关节位置轨迹（前4个状态变量）
plt.subplot(2, 1, 1)
for i in range(num_joints):
    # 获取参考状态，注意我们在仿真时要按照时间 t 计算不同的参考值
    ref_trajectory = np.array([get_x_ref(t)[i] for t in time_hist])  # 提取 q_ref 部分
    
    plt.plot(time_hist, ref_trajectory, label=f"关节 {i+1} 参考位置")

plt.xlabel("时间(s)")
plt.ylabel("关节角 (rad)")
plt.title("双臂机器人各关节位置轨迹")
plt.legend()
plt.grid(True)
plt.savefig("mpc_dualarm_xd.eps", format="eps")
plt.show()

# 绘制控制输入轨迹
plt.subplot(2, 1, 2)
for i in range(m):
    plt.step(time_hist[:-1], u_hist[:, i], where='post', label="关节 %d力矩" % (i+1))
plt.xlabel("时间(s)")
plt.ylabel("控制输入 (N·m)")
plt.title("双臂机器人控制输入")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("mpc_dualarm_u.eps", format="eps")
plt.show()

#============================

# 4. 额外：验证协同约束
#============================

# 检查每时刻左臂第一个关节与右臂第一个关节位置差异
collab_diff = x_hist[:, 0] - x_hist[:, 2]
plt.figure()
plt.plot(time_hist, collab_diff, label="q_left1 - q_right1")
plt.hlines([delta - epsilon, delta + epsilon], xmin=time_hist[0], xmax=time_hist[-1], colors='r', linestyles='dashed', label="±ε")
plt.xlabel("时间(s)")
plt.ylabel("位置差 (rad)")
plt.title("协同约束验证：左臂第1关节与右臂第1关节位置差")
plt.legend()
plt.grid(True)
plt.savefig("collab_constraint.eps", format="eps")
plt.show()

# 结束
print("仿真结束。")