# ROV Pixhawk 控制节点 — 配置与操作指南

> **适用硬件**：地瓜派 X5 + Pixhawk 飞控 + 游戏手柄  
> **适用 ROV**：6 推进器（4 垂直带 ±30° 倾角 + 2 水平前进）

---

## 目录

- [1. 系统架构概述](#1-系统架构概述)
  - [1.3 操作模式与按键映射](#13-操作模式与按键映射)
  - [1.5 姿态数据上报节点](#15-姿态数据上报节点attitude_publisher)
- [2. 硬件接线](#2-硬件接线)
- [3. 软件环境搭建](#3-软件环境搭建)
- [4. 无硬件纯软件验证](#4-无硬件纯软件验证)
  - [测试 12-16：模式切换、灯/舵机控制](#测试-12模式切换--mode-1辅助设备)
  - [测试 17：姿态上报节点](#测试-17姿态上报节点)
- [5. 参数参考手册](#5-参数参考手册)
  - [5.6 辅助设备参数](#56-辅助设备参数新增)
  - [5.7 姿态上报节点参数](#57-姿态上报节点参数attitude_publisher)
- [6. 调参指南](#6-调参指南)
- [7. 故障排查](#7-故障排查)

---

## 1. 系统架构概述

### 1.1 ROV 推进器排布

```
                         ↑ 前进方向 (Surge +X)
                         |
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        │   T1 (Ch4)      │   T2 (Ch5)       │  ← 前方一对
        │  左前·内倾30°   │  右前·内倾30°    │     垂直推进器
        │   ↙ 向内侧      │   ↘ 向内侧       │     (推力含垂直+水平分量)
        │                 │                 │
        │                 │                 │
        │   T5 (Ch8)      │   T6 (Ch9)       │  ← 前进推进器
        │  左前进·水平     │  右前进·水平     │     (水平推力)
        │   →             │   →             │
        │                 │                 │
        │   T3 (Ch6)      │   T4 (Ch7)       │  ← 后方一对
        │  左后·外倾30°   │  右后·外倾30°    │     垂直推进器
        │   ↖ 向外侧      │   ↗ 向外侧       │     (推力含垂直+水平分量)
        │                 │                 │
        └─────────────────┼─────────────────┘
                          │
                    尾部 (Heave +Z 向上)
```

| 编号 | Channel | 位置 | 类型 | 倾角 |
|------|---------|------|------|------|
| T1 | 4 | 左前 | 垂直 | 内倾 30°（向内侧推） |
| T2 | 5 | 右前 | 垂直 | 内倾 30°（向内侧推） |
| T3 | 6 | 左后 | 垂直 | 外倾 30°（向外侧推） |
| T4 | 7 | 右后 | 垂直 | 外倾 30°（向外侧推） |
| T5 | 8 | 左 | 前进 | 水平（向前推） |
| T6 | 9 | 右 | 前进 | 水平（向前推） |

> **倾斜推力的水平分量为什么不会互相干扰？**  
> 前向内倾 + 后向外倾的对称布局，使得 Heave/Pitch/Roll 操作时水平分力在 4 推进器间自动抵消（推导见代码注释或计划附录）。

### 1.2 信号流

```
┌──────────┐    USB     ┌───────────────────────────────────────────────┐
│ Xbox 手柄 │ ────────→  │              地瓜派 X5                        │
│          │   /joy     │                                               │
└──────────┘            │  ┌─────────────────────────────────────┐      │
                        │  │ rov_pixhawk_control_node             │      │
                        │  │                                     │      │
                        │  │ 摇杆 axes ──→ 推进器混控 (始终)       │      │
                        │  │ 按键 btn  ──→ 模式切换 + 灯/舵机      │      │
                        │  │             (Mode 1: 辅助设备)       │      │
                        │  └────────────────┬────────────────────┘      │
                        │                   │ /mavros/rc/override       │
                        └───────────────────┼───────────────────────────┘
                                            │ MAVLink
                                            ▼
                                    ┌──────────────┐    PWM      ┌──────────────┐
                                    │   Pixhawk    │ ────────→  │ 6×推进器     │
                                    │   飞控       │  1100~      │ + 灯 + 舵机  │
                                    │              │  1900μs     │ (ch13-14)    │
                                    └──────┬───────┘             └──────────────┘
                                           │
                                           │ /mavros/imu/data
                                           ▼
              ┌────────────────────────────────────────────┐
              │  地瓜派 X5                                  │
              │                                            │
              │  ┌──────────────────────────────────┐      │
              │  │ attitude_publisher_node (独立)     │      │
              │  │                                  │      │
              │  │ IMU → 欧拉角 → /rov/attitude     │      │
              │  │              → UDP :5005 (JSON)  │      │
              │  │              → CSV 日志           │      │
              │  └──────────────────────────────────┘      │
              └────────────────────────────────────────────┘
```

> **两个节点**：
> - `rov_pixhawk_control_node`：推进器 + 模式切换 + 灯/舵机控制（发布 PWM）
> - `attitude_publisher_node`：姿态数据上报（ROS2 topic + UDP + CSV），与推进器控制完全解耦

### 1.3 操作模式与按键映射

#### 1.3.1 模式切换

节点运行时有 **4 个操作模式**，按 Xbox 手柄的 **`Back` 键**(buttons[6]) 循环切换：

| 模式 | 名称 | 进入方式 | 功能 |
|------|------|---------|------|
| **Mode 0** | 手动控制 | 启动默认 | 摇杆 → 推进器直驱；按键未映射 |
| **Mode 1** | 辅助设备 | 按 `Back` 1 次 | 灯开关 + 舵机控制（见下表） |
| **Mode 2** | 定姿保持 | 按 `Back` 2 次 | 预留——摇杆松手后 PID 保持姿态 |
| **Mode 3** | 定深保持 | 按 `Back` 3 次 | 预留——摇杆松手后 PID 保持深度 |

> 当前模式会打印在 ROS 日志中，也会记录到 CSV 的 `Mode` 列。

#### 1.3.2 摇杆 → 自由度映射（所有模式通用）

**摇杆 axes 在所有模式下始终控制推进器，不受模式切换影响：**

```
左摇杆 上下 ──→ Surge  (进退)
左摇杆 左右 ──→ Yaw    (转艏)
右摇杆 上下 ──→ Pitch  (俯仰)
右摇杆 左右 ──→ Roll   (横滚)
LT 扳机     ──→ Heave+ (上浮)
RT 扳机     ──→ Heave- (下潜)
```

#### 1.3.3 Mode 1（辅助设备）按键映射

仅在 Mode 1 下生效：

| Xbox 按键 | buttons 索引 | 功能 | 触发方式 |
|-----------|-------------|------|---------|
| **A** | 0 | 灯 1 开/关 (PWM 1100↔1900) | 按下瞬间翻转 |
| **B** | 1 | 灯 2 开/关 (PWM 1100↔1900) | 按下瞬间翻转 |
| **X** | 2 | 舵机正转 | **按住不放**持续转动，松开即停 |
| **Y** | 3 | 舵机反转 | **按住不放**持续转动，松开即停 |
| **LB** | 4 | 舵机回中位 (PWM=1500) | 按下瞬间触发 |
| **Back** | 6 | 切换到下一模式 | 按下瞬间切换 |

> **舵机控制逻辑**：
> - 按住 X/Y → 每帧 PWM ±5μs（约 100μs/s）
> - 松开 → PWM 保持当前值，舵机停在该位置
> - LB 快捷回中
> - PWM 自动限幅 [1100, 1900]

#### 1.3.4 灯/舵机 PWM 通道

| 设备 | MAIN OUT | MAVLink Index | PWM 值 |
|------|----------|---------------|--------|
| 舵机 | 13 | channels[12] | 1100~1900（中位 1500） |
| 灯 1 | 14 | channels[13] | 1100=灭, 1900=亮 |
| 灯 2 | 14 | channels[13] | 暂共用 ch13，后续可扩展 |

> ⚠️ Pixhawk 固件通道 1-3 为固定功能，不可用。空闲通道从 MAIN OUT 13 开始。

### 1.4 推进器分配矩阵

```
前进推进器:
  T5 (Ch8) = Surge + Yaw      (左前进，Yaw 右转时减弱)
  T6 (Ch9) = Surge - Yaw      (右前进，Yaw 右转时增强)

垂直推进器:
  T1 (Ch4) = Heave + Pitch + Roll   (左前)
  T2 (Ch5) = Heave + Pitch - Roll   (右前)
  T3 (Ch6) = Heave - Pitch + Roll   (左后)
  T4 (Ch7) = Heave - Pitch - Roll   (右后)
```

> **Heave 符号约定**：正值 = 推力向上（上浮），负值 = 推力向下（下潜）。

### 1.5 姿态数据上报节点（attitude_publisher）

独立于推进器控制的姿态上报节点，将 Pixhawk 内置 IMU 的姿态数据实时传输到上位机。

```
Pixhawk IMU → MAVROS → /mavros/imu/data
                              ↓
                    ┌────────────────────────────┐
                    │ attitude_publisher_node     │
                    │                            │
                    │ 四元数 → 欧拉角 (Roll/Pitch/Yaw) │
                    │                            │
                    ├─ /rov/attitude (ROS2)       │ → 上位机 ROS2 订阅
                    ├─ UDP :5005 广播 (JSON)      │ → 上位机自定义 GUI
                    └─ CSV 日志                   │ → 本地离线分析
                    └────────────────────────────┘
```

**三种数据输出方式：**

| 输出 | 格式 | 位置 | 适用场景 |
|------|------|------|---------|
| **ROS2 Topic** | `geometry_msgs/Vector3Stamped` | `/rov/attitude` | 上位机 ROS2 节点订阅；可用 `ros2 bag record` 录制回放 |
| **UDP 广播** | JSON `{"ts":...,"roll":...,"pitch":...,"yaw":...,"roll_rad":...,"pitch_rad":...,"yaw_rad":...}` | `255.255.255.255:5005` | 非 ROS 上位机（Qt/Web/Unity 自定义 GUI） |
| **CSV 日志** | `Timestamp,Roll(deg),Pitch(deg),Yaw(deg),Roll(rad),Pitch(rad),Yaw(rad)` | `rov_attitude_log_YYYYMMDD_HHMMSS.csv` | Excel / Python pandas / MATLAB 离线分析 |

**后续处理示例：**

```python
# 方式 1: pandas 读取 CSV
import pandas as pd
df = pd.read_csv('rov_attitude_log_20260715_143022.csv')
df.plot(x='Timestamp', y=['Roll(deg)', 'Pitch(deg)', 'Yaw(deg)'])

# 方式 2: ros2 bag 录制（推荐用于完整数据记录）
ros2 bag record /rov/attitude /mavros/imu/data

# 方式 3: UDP 接收端（上位机 Python 示例）
import socket, json
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5005))
while True:
    data, addr = sock.recvfrom(1024)
    attitude = json.loads(data.decode())
    print(f"Roll={attitude['roll']:.1f} Pitch={attitude['pitch']:.1f} Yaw={attitude['yaw']:.1f}")
```

> **推荐**：实际作业时用 `ros2 bag record /rov/attitude /mavros/imu/data` 录制全部话题，可完整回放所有传感器数据。CSV 作为轻量备选方便快速查看。

---

## 2. 硬件接线

### 2.1 连接总览

```
┌─────────────────────────────────────────────────────────────┐
│                        地瓜派 X5                            │
│                                                             │
│  USB-A ──── 游戏手柄                                        │
│  USB-C / micro-USB ──── Pixhawk (USB 口)                    │
│                     (或 UART TX/RX → Pixhawk TELEM2)        │
│  Ethernet / WiFi ──── 远程 PC (SSH 调试) [可选]             │
│  电源 12V DC                                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                        Pixhawk                              │
│                                                             │
│  USB / TELEM2 ←── 地瓜派 X5 (MAVLink 通信)                   │
│  MAIN OUT 1~6  ──→ 6 路电调 → 6 个推进器                     │
│  POWER         ←── 电池 / BEC                               │
│  GPS/罗盘       ──  外置模块 [ROV 水下无法用 GPS，可省略]     │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 地瓜派 ↔ Pixhawk 连接

| 方式 | Pixhawk 接口 | 地瓜派接口 | 参数 | 推荐度 |
|------|-------------|-----------|------|--------|
| **USB** | USB 口 | USB-A | `fcu_url:=/dev/ttyACM0:921600` | ⭐ 最简单，推荐调试用 |
| **串口** | TELEM2 | UART (TX/RX) | `fcu_url:=/dev/ttyS1:921600` | ⭐⭐ 量产方案，更可靠 |

> ⚠️ **重要**：Pixhawk 通过 USB 连接时不要同时接电池供电（部分型号 USB 和电池冲突），或者先接电池再接 USB（视飞控型号而定）。

### 2.3 游戏手柄

直接 USB 插入地瓜派即可。Linux 内核自带 `joydev` 驱动，设备路径通常为 `/dev/input/js0`。

验证手柄是否被识别：
```bash
ls /dev/input/js*
# 输出: /dev/input/js0

# 查看按键/轴事件
sudo apt install joystick -y
jstest /dev/input/js0
```

### 2.4 Pixhawk PWM → 推进器

| Pixhawk MAIN OUT | 推进器 | 功能 |
|------------------|--------|------|
| MAIN 4 | T1 左前垂直 | 垂直+水平（内倾30°） |
| MAIN 5 | T2 右前垂直 | 垂直+水平（内倾30°） |
| MAIN 6 | T3 左后垂直 | 垂直+水平（外倾30°） |
| MAIN 7 | T4 右后垂直 | 垂直+水平（外倾30°） |
| MAIN 8 | T5 左前进 | 前进 |
| MAIN 9 | T6 右前进 | 前进 |

> **Pixhawk 的 MAIN OUT 通道编号是从 1 开始的**，对应 MAVLink `RC_CHANNELS_OVERRIDE` 消息的 `chan4_raw` ~ `chan9_raw`（注意 MAVLink 内部是从 0 开始索引，代码中 `channels[4]` 对应 MAIN 5）。

### 2.5 Pixhawk RC_OVERRIDE 参数设置

在 QGroundControl 或 Mission Planner 中确保以下参数正确：

| 参数 | 值 | 说明 |
|------|-----|------|
| `SYSID_MYGCS` | 1 | 本机（地瓜派）MAVLink ID |
| `SER_TEL2_BAUD` | 921600 | TELEM2 波特率（如用串口连接） |
| `RC_MAP_*` | 默认 | RC 通道映射（不影响 override） |

> 使用 RC Override 方式控制推进器**不需要**在飞控上设置任何混控（mixer），因为混控逻辑已经在本 ROS2 节点的分配矩阵中完成。飞控只需把 MAIN OUT 1~9 设为直通 PWM 输出即可。

---

## 3. 软件环境搭建

### 3.1 WSL2 纯软件验证（无硬件）— 最小环境

如果只在 WSL2 中做纯软件验证（不接飞控和手柄），只需安装最少依赖：

```bash
# === Humble (Ubuntu 22.04) ===
sudo apt install ros-humble-ros-base python3-colcon-common-extensions -y
sudo apt install ros-humble-mavros-msgs ros-humble-sensor-msgs -y

# === Jazzy (Ubuntu 24.04) ===
sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions -y
sudo apt install ros-jazzy-mavros-msgs ros-jazzy-sensor-msgs -y
```

> **如果 `mavros-msgs` 找不到**（部分 ROS2 发行版未收录此包），用源码编译绕过：
> ```bash
> cd ~/rov_ws/src
> git clone https://github.com/mavlink/mavros.git -b ros2
> cd ~/rov_ws
> source /opt/ros/${ROS_DISTRO}/setup.bash
> colcon build --packages-select mavros_msgs
> source install/setup.bash
> ```

### 3.2 地瓜派 / Linux 开发机（完整硬件环境）

```bash
# 1. 安装 MAVROS 完整版
sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras -y

# 2. 安装 GeographicLib 数据集（MAVROS 需要）
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh

# 3. 安装 joystick 相关
sudo apt install joystick jstest-gtk -y
```

### 3.3 从 GitHub 克隆（地瓜派 / 新机器）

如果从零开始在地瓜派或新 Linux 机器上部署：

```bash
# 1. 创建 ROS2 工作空间
mkdir -p ~/rov_ws/src
cd ~/rov_ws/src

# 2. 克隆 package
git clone https://github.com/Crysknife07/rov_ws.git rov_pixhawk_control
# 注意: 仓库 URL 名称是 rov_ws，但实际只包含 package，
#       克隆时重命名为 rov_pixhawk_control 放到 src/ 下

# 3. 安装依赖（如果尚未安装）
sudo apt install ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-sensor-msgs -y

# 4. 编译
cd ~/rov_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select rov_pixhawk_control
source install/setup.bash

# 5. 验证
ros2 run rov_pixhawk_control rov_pixhawk_control_node --help
ros2 run rov_pixhawk_control attitude_publisher --help
```

> ⚠️ **重要**：仓库名叫 `rov_ws` 但只包含 package 本身（没有完整 ROS2 工作空间的 `build/`、`install/`、`src/` 目录）。克隆时务必放到你的工作空间的 `src/` 下并重命名为 `rov_pixhawk_control`。

### 3.4 编译本节点（已有本地代码）

```bash
# === WSL2：先把 Windows 上的代码拷贝到 WSL2 原生文件系统 ===
mkdir -p ~/rov_ws/src
cp -r /mnt/e/desktop/rov_ws/rov_ws/rov_pixhawk_control ~/rov_ws/src/

# === 编译 ===
cd ~/rov_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select rov_pixhawk_control
source install/setup.bash
```

> ⚠️ **WSL2 用户注意**：不要直接在 `/mnt/e/...` 路径下编译，I/O 性能差且可能产生权限问题。始终拷贝到 `~/` 原生文件系统后再编译。

### 3.5 连接硬件时的启动顺序

```bash
# === 终端 1：启动 MAVROS ===
ros2 launch mavros px4.launch.py fcu_url:=/dev/ttyACM0:921600
# 如果通过 TELEM2 串口连接：
# ros2 launch mavros px4.launch.py fcu_url:=/dev/ttyS1:921600

# === 终端 2：确认 MAVROS 连接正常 ===
ros2 topic list | grep mavros
# 应看到 /mavros/state, /mavros/rc/override, /mavros/imu/data 等话题

# 检查飞控是否连接
ros2 topic echo /mavros/state --once
# connected: True ← 确认此项为 True

# === 终端 3：启动 ROV 控制节点（推进器 + 灯/舵机）===
ros2 run rov_pixhawk_control rov_pixhawk_control_node

# === 终端 4：启动姿态上报节点 ===
ros2 run rov_pixhawk_control attitude_publisher

# === 终端 5（可选）：启动 joy 节点（将手柄原始数据转成 /joy 话题）===
ros2 run joy joy_node
# 或
ros2 launch rov_pixhawk_control joy.launch.py  # 如果有的话
```

> ⚠️ **注意**：`joy_node` 是 ROS2 自带的 `joy` 包中的节点。如果未安装：`sudo apt install ros-${ROS_DISTRO}-joy -y`

---

## 4. 无硬件纯软件验证（WSL2 / 任意 Linux）

> ✅ **已验证环境**：WSL2 + ROS2 Jazzy，无需飞控、无需手柄、无需 MAVROS。

不连接任何硬件也能验证节点的核心逻辑：用 `ros2 topic pub` 模拟手柄输入，用 `ros2 topic echo` 观察 PWM 输出。

### 4.1 启动节点

开**终端 A**：

```bash
cd ~/rov_ws
source install/setup.bash
ros2 run rov_pixhawk_control rov_pixhawk_control_node
```

正常输出：
```
[INFO] [...] ROV node active, logging to: /home/ubuntu/rov_imu_log_20260715_....csv
```

### 4.2 启动 PWM 监听

开**终端 B**：

```bash
cd ~/rov_ws
source install/setup.bash
ros2 topic echo /mavros/rc/override
```

> 终端 A 启动后终端 B 才能成功 echo。如果终端 B 显示 `does not appear to be published yet`，说明终端 A 还没启动成功。

### 4.3 发送模拟手柄指令

开**终端 C**，执行以下命令。每执行一条，观察终端 B 的 PWM 变化。

> ⚠️ **YAML 格式要点**：ROS2 要求用 `{...}` 包裹，且 `axes:` 后必须有空格：
> ```bash
> # ✅ 正确
> ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
>
> # ❌ 错误（会报 "needs to be a dictionary in YAML format"）
> ros2 topic pub --once /joy sensor_msgs/msg/Joy "axes: [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
> ```

#### 测试 1：中位（所有杆归零）

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**预期**：channels[4]~[9] 全部 **1500**，其余通道 65535。

---

#### 测试 2：全速前进（左杆推到最上方）

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

| Ch4 | Ch5 | Ch6 | Ch7 | Ch8 | Ch9 |
|-----|-----|-----|-----|-----|-----|
| 1500 | 1500 | 1500 | 1500 | **1900** | **1900** |

---

#### 测试 3：全速后退（左杆拉到最下方）

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

| Ch8 | Ch9 |
|-----|-----|
| **1100** | **1100** |

---

#### 测试 4：右转（左杆推到最右）

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]}"
```

| Ch8 (T5 左) | Ch9 (T6 右) |
|-------------|-------------|
| **1100** | **1900** |

---

#### 测试 5：左转（左杆推到最左）

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0]}"
```

| Ch8 (T5 左) | Ch9 (T6 右) |
|-------------|-------------|
| **1900** | **1100** |

---

#### 测试 6：低头（右杆推到最上）

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]}"
```

| Ch4 (T1 左前) | Ch5 (T2 右前) | Ch6 (T3 左后) | Ch7 (T4 右后) |
|---------------|---------------|---------------|---------------|
| **1900** | **1900** | **1100** | **1100** |

---

#### 测试 7：仰头（右杆拉到最下）

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0]}"
```

| Ch4 (T1) | Ch5 (T2) | Ch6 (T3) | Ch7 (T4) |
|----------|----------|----------|----------|
| **1100** | **1100** | **1900** | **1900** |

---

#### 测试 8：右翻（右杆推到最右）

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]}"
```

| Ch4 (T1 左前) | Ch5 (T2 右前) | Ch6 (T3 左后) | Ch7 (T4 右后) |
|---------------|---------------|---------------|---------------|
| **1900** | **1100** | **1900** | **1100** |

---

#### 测试 9：左翻（右杆推到最左）

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0]}"
```

| Ch4 (T1) | Ch5 (T2) | Ch6 (T3) | Ch7 (T4) |
|----------|----------|----------|----------|
| **1100** | **1900** | **1100** | **1900** |

---

#### 测试 10：上浮（LT 按到底）

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0]}"
```

| Ch4 | Ch5 | Ch6 | Ch7 |
|-----|-----|-----|-----|
| **1900** | **1900** | **1900** | **1900** |

---

#### 测试 11：下潜（RT 按到底）

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]}"
```

| Ch4 | Ch5 | Ch6 | Ch7 |
|-----|-----|-----|-----|
| **1100** | **1100** | **1100** | **1100** |

---

#### 测试 12：模式切换 → Mode 1（辅助设备）

```bash
# 模拟按 Back 键 (buttons[6]=1)
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0,0,0,0,0,0,1,0,0,0,0]}"
```

**预期**：终端 A 日志输出 `Mode → 1 (Auxiliary (Lights & Servo))`。

---

#### 测试 13：Mode 1 — 开灯 1

```bash
# 确保已在 Mode 1（先执行测试 12），然后按 A (buttons[0]=1)
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [1,0,0,0,0,0,0,0,0,0,0]}"
```

**预期**：channels[13] = **1900**（灯亮），日志 `Light 1 → ON`。

---

#### 测试 14：Mode 1 — 关灯 1

```bash
# 再次按 A
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [1,0,0,0,0,0,0,0,0,0,0]}"
```

**预期**：channels[13] = **1100**（灯灭），日志 `Light 1 → OFF`。

---

#### 测试 15：Mode 1 — 舵机正转（按住 X）

```bash
# 按住 X (buttons[2]=1) — 每发一次 channels[12] 增加 5μs
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0,0,1,0,0,0,0,0,0,0,0]}"
# 连续多次发送模拟"按住"效果
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0,0,1,0,0,0,0,0,0,0,0]}"
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0,0,1,0,0,0,0,0,0,0,0]}"
```

**预期**：channels[12] 每帧递增（1505 → 1510 → 1515）。

---

#### 测试 16：Mode 1 — 舵机回中（按 LB）

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0,0,0,0,1,0,0,0,0,0,0]}"
```

**预期**：channels[12] = **1500**，日志 `Servo → centre`。

---

#### 测试 17：姿态上报节点

```bash
# 终端 D: 启动姿态节点
ros2 run rov_pixhawk_control attitude_publisher

# 终端 E: 监听 ROS2 话题
ros2 topic echo /rov/attitude

# 终端 F: 模拟 IMU 数据（45° 偏航）
ros2 topic pub --once /mavros/imu/data sensor_msgs/msg/Imu "{orientation: {x: 0.0, y: 0.0, z: 0.3827, w: 0.9239}}"

# 终端 F: 监听 UDP（如有网络）
# nc -u -l 5005
```

**预期**：
- `/rov/attitude` 输出 `vector: {x: ~0.0, y: ~0.0, z: ~45.0}`
- UDP 收到 JSON：`{"ts":...,"roll":...,"pitch":...,"yaw":45.0,...}`
- 生成 `rov_attitude_log_*.csv` 文件

---

### 4.4 停止节点

在终端 A 按 `Ctrl+C`，正常输出：
```
[INFO] [...] CSV log saved and closed
```

### 4.5 一键验证脚本

将以下脚本保存为 `~/rov_ws/test_alloc.sh`，一次性跑完所有测试：

```bash
#!/bin/bash
# ROV 推进器分配矩阵 — 一键验证脚本
# 使用前先启动节点: ros2 run rov_pixhawk_control rov_pixhawk_control_node

source /opt/ros/${ROS_DISTRO}/setup.bash 2>/dev/null
source ~/rov_ws/install/setup.bash 2>/dev/null

TESTS=(
  "中位:0.0,0.0,0.0,0.0,0.0,0.0,0.0"
  "全速前进:0.0,-1.0,0.0,0.0,0.0,0.0,0.0"
  "全速后退:0.0,1.0,0.0,0.0,0.0,0.0,0.0"
  "右转:0.0,0.0,1.0,0.0,0.0,0.0,0.0"
  "左转:0.0,0.0,-1.0,0.0,0.0,0.0,0.0"
  "低头:0.0,0.0,0.0,1.0,0.0,0.0,0.0"
  "仰头:0.0,0.0,0.0,-1.0,0.0,0.0,0.0"
  "右翻:0.0,0.0,0.0,0.0,1.0,0.0,0.0"
  "左翻:0.0,0.0,0.0,0.0,-1.0,0.0,0.0"
  "上浮:0.0,0.0,0.0,0.0,0.0,-1.0,0.0"
  "下潜:0.0,0.0,0.0,0.0,0.0,0.0,-1.0"
)

for t in "${TESTS[@]}"; do
  name="${t%%:*}"
  axes="${t##*:}"
  echo "=== $name ==="
  ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [$axes]}" 2>/dev/null
  sleep 0.1
  ros2 topic echo /mavros/rc/override --once --field channels 2>/dev/null | head -10
  echo ""
done
```

```bash
chmod +x ~/rov_ws/test_alloc.sh
./test_alloc.sh
```

### 4.6 WSL2 常见踩坑

| 症状 | 原因 | 解决 |
|------|------|------|
| `Unable to locate package ros-*-mavros-msgs` | 该发行版未收录 mavros_msgs | 用源码编译（见 3.1 节绕过方案） |
| `needs to be a dictionary in YAML format` | `ros2 topic pub` 的数值参数需要 `{...}` 包裹 | 加花括号：`"{axes: [...]}"` |
| `does not appear to be published yet` | 节点未启动，没有发布者 | 先在终端 A 启动节点 |
| 编译报 `ModuleNotFoundError: mavros_msgs` | 未 source 编译后的 mavros_msgs | `source install/setup.bash` |
| `/mnt/e/` 下编译极慢 | WSL2 跨文件系统 I/O 性能差 | 始终拷贝到 `~/` 再编译 |

---

## 5. 参数参考手册

所有可调参数集中在 [rov_pixhawk_control.py](rov_pixhawk_control/rov_pixhawk_control.py) 的 `__init__` 方法中，以 `self.cfg_` 前缀命名。

### 5.1 摇杆轴映射

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cfg_axis_surge` | 1 | 进退轴 index（左杆上下） |
| `cfg_axis_yaw` | 2 | 转艏轴 index（左杆左右） |
| `cfg_axis_pitch` | 3 | 俯仰轴 index（右杆上下） |
| `cfg_axis_roll` | 4 | 横滚轴 index（右杆左右） |
| `cfg_axis_heave_up` | 5 | 上浮轴 index（LT 扳机） |
| `cfg_axis_heave_down` | 6 | 下潜轴 index（RT 扳机） |

> **如何确定你的手柄轴编号？**  
> ```bash
> # 方法 1：用 jstest 观察
> jstest /dev/input/js0
> # 动一动对应的摇杆/扳机，观察 "Axes" 行中哪个序号的值变化
>
> # 方法 2：用 ROS 观察原始 joy 数据
> ros2 topic echo /joy --once
> # 动摇杆后再次执行，对比 axes 数组的变化
> ```
> Joy 消息的 `axes[]` 索引从 0 开始。`cfg_axis_surge = 1` 对应 `axes[1]`。

### 5.2 PWM 通道映射

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cfg_chn_vert_fl` | 4 | T1 垂直左前 → MAVLink Channel 4 |
| `cfg_chn_vert_fr` | 5 | T2 垂直右前 → MAVLink Channel 5 |
| `cfg_chn_vert_rl` | 6 | T3 垂直左后 → MAVLink Channel 6 |
| `cfg_chn_vert_rr` | 7 | T4 垂直右后 → MAVLink Channel 7 |
| `cfg_chn_fwd_l` | 8 | T5 前进左 → MAVLink Channel 8 |
| `cfg_chn_fwd_r` | 9 | T6 前进右 → MAVLink Channel 9 |

> **MAVLink Channel 编号说明**：`msg_rc.channels[i]` 的 `i` 是 **0 起始索引**。Pixhawk MAIN OUT 的编号 = `i + 1`。例如 `channels[4]` 对应 MAIN OUT 5。配置时注意核对飞控的 SERVO_MAIN_FUNCTION 参数。

### 5.3 混合增益

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `cfg_gain_surge` | 1.0 | 0.0 ~ 1.0 | 进退响应强度 |
| `cfg_gain_yaw` | 1.0 | 0.0 ~ 1.0 | 转艏响应强度 |
| `cfg_gain_pitch` | 1.0 | 0.0 ~ 1.0 | 俯仰响应强度 |
| `cfg_gain_roll` | 1.0 | 0.0 ~ 1.0 | 横滚响应强度 |
| `cfg_gain_heave` | 1.0 | 0.0 ~ 1.0 | 升降响应强度 |

> 减小增益可以让该自由度的操控更"柔和"，适合精细操控场景。

### 5.4 PWM 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cfg_pwm_center` | 1500 | PWM 中位值（μs），推进器停转 |
| `cfg_pwm_range` | 400 | PWM 变化范围（中位 ±400 = 1100~1900） |
| `cfg_pwm_min` | 1100 | PWM 下限（μs） |
| `cfg_pwm_max` | 1900 | PWM 上限（μs） |
| `cfg_dead_zone` | 0.05 | 死区阈值（5%），摇杆在此范围内视为 0 |

### 5.5 其他参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cfg_log_interval` | 10 | 每 N 帧记录一次 IMU 姿态到 CSV |

### 5.6 辅助设备参数（新增）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cfg_servo_channel` | 12 | 舵机 PWM 通道（MAIN OUT 13） |
| `cfg_light1_channel` | 13 | 灯 1 PWM 通道（MAIN OUT 14） |
| `cfg_light2_channel` | 13 | 灯 2 PWM 通道（暂共用 ch13，后续可改 14） |
| `cfg_servo_rate` | 5 | 舵机转速（每帧 PWM 变化 μs，~5μs/frame） |
| `cfg_servo_pwm_min` | 1100 | 舵机 PWM 下限 |
| `cfg_servo_pwm_max` | 1900 | 舵机 PWM 上限 |

### 5.7 姿态上报节点参数（attitude_publisher）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cfg_publish_rate` | 50.0 | ROS2 / UDP 发布频率（Hz） |
| `cfg_enable_udp` | True | 是否启用 UDP 广播 |
| `cfg_udp_host` | 255.255.255.255 | UDP 目标地址（广播） |
| `cfg_udp_port` | 5005 | UDP 目标端口 |
| `cfg_csv_interval` | 10 | 每 N 帧写一行 CSV |

---

## 6. 调参指南

### 6.1 死区调整

**症状**：摇杆松开后推进器仍有微弱的嗡嗡声 / PWM 值在 1500 附近抖动。

**操作**：增大 `cfg_dead_zone`，例如从 0.05 → 0.08。

```python
self.cfg_dead_zone = 0.08   # 原 0.05，摇杆抖动大的手柄可调到 0.10
```

> 注意：死区过大会导致摇杆前段"没反应"，操控感觉迟钝。一般 0.05~0.10 是合理范围。

### 6.2 增益调整

**症状**：某个自由度的推进器出力太猛或太弱。

**操作**：调整对应增益。

```python
self.cfg_gain_pitch = 0.6   # 俯仰太猛，降低到 60%
self.cfg_gain_heave = 0.8   # 升降稍弱，提高到 80%
```

> 建议在水池/浅水区测试，逐个自由度独立测试。先设所有 gain = 0.5，逐步调到满意的响应。

### 6.3 PWM 范围调整

**症状**：推进器在 PWM=1100 时仍在反转，或 PWM=1900 时未达到最大推力。

**操作**：

- 如果推进器电调支持更宽的 PWM 范围（如 1000~2000）：
  ```python
  self.cfg_pwm_min = 1000
  self.cfg_pwm_max = 2000
  self.cfg_pwm_range = 500  # 中位 1500 ± 500
  ```

- 如果推进器是单向的（不支持反转，只能正转调速）：
  ```python
  self.cfg_pwm_min = 1500   # 小于 1500 视为 0 推力
  # 并在 cmd_to_pwm 函数中调整逻辑（需要修改代码）
  ```

### 6.4 反转推进器方向

**症状**：推"前进"杆 ROV 却后退。

**操作**：将对应增益设为负值。

```python
self.cfg_gain_surge = -1.0  # 反转进退方向
```

或者在混合矩阵中调换对应通道的符号（需修改 joy_callback 中的混合逻辑）。

### 6.5 调参流程建议

```
第1步（陆上无桨）：确认 PWM 通道方向正确
  └→ 逐自由度推到极限，观察推进器转向是否符合预期

第2步（浅水池）：调整增益
  └→ 先调 surge/heave（基础运动），再调 yaw/pitch/roll（姿态）

第3步（开放水域）：微调
  └→ 实测各自由度响应，精调增益和死区
```

### 6.6 摇杆轴方向符号

不同手柄的摇杆轴方向约定可能不同。常见情况：

| 手柄品牌 | 左杆上推 | 左杆右推 | LT 按下 |
|----------|---------|---------|---------|
| Xbox | axes[1] = -1.0 | axes[2] = +1.0 | axes[5] = -1.0 |
| PS4/PS5 | axes[1] = -1.0 | axes[2] = +1.0 | axes[5] = -1.0 |
| 北通/罗技 | 因型号而异 | 需实测 | 需实测 |

> **实测方法**：运行 `ros2 topic echo /joy`，操作摇杆观察 `axes[]` 值的变化方向和范围。然后根据实测值调整 `cfg_axis_*` 和增益符号。

---

## 7. 故障排查

### 7.1 节点启动失败

| 症状 | 可能原因 | 解决方法 |
|------|---------|---------|
| `ModuleNotFoundError: rclpy` | 未 source ROS2 环境 | `source /opt/ros/${ROS_DISTRO}/setup.bash` |
| `ModuleNotFoundError: mavros_msgs` | mavros_msgs 未安装 | `sudo apt install ros-${ROS_DISTRO}-mavros-msgs -y` |
| `ImportError: cannot import name 'Node' from 'rclpy'` | 旧版 ROS2 / 代码问题 | 确保使用 `from rclpy.node import Node`（已修复） |
| 启动后直接退出，无报错 | `joy_node` 未运行，/joy 话题无消息 | 先启动 `ros2 run joy joy_node` |
| `CSV log saved and closed` 即退出 | KeyboardInterrupt 正常退出 | 这是正常关闭流程 |

### 7.2 手柄无响应

```bash
# 1. 确认设备存在
ls /dev/input/js*
# 如果没有输出 → 手柄未插入或驱动未加载

# 2. 确认 joy_node 在运行
ros2 node list | grep joy
# 如果没有 → ros2 run joy joy_node

# 3. 确认 /joy 话题有数据
ros2 topic echo /joy --once
# 动一动摇杆再试一次，看数据是否变化

# 4. 确认节点的轴映射编号正确
# 对比 ros2 topic echo /joy 输出的 axes 数组与你 cfg_axis_* 的值
```

### 7.3 推进器不转

```bash
# 1. 确认 Pixhawk 已连接且 MAVROS 正常
ros2 topic echo /mavros/state --once | grep connected
# connected: True ← 必须为 True

# 2. 确认 override 消息在发布
ros2 topic echo /mavros/rc/override --once
# 观察 channels 数组，非 65535 的通道就是被 override 的通道

# 3. 确认飞控已 arm
ros2 topic echo /mavros/state --once | grep armed
# armed: True ← 某些飞控需要 arm 后才输出 PWM

# 4. 在 QGroundControl 中检查 SERVO 输出
# 连接 QGC → Setup → Actuators → 查看 MAIN 1~9 的输出值
```

### 7.4 飞控拒绝 RC Override

```bash
# 检查 MAVROS 是否正常收发
ros2 topic hz /mavros/rc/override
# 应该看到 ~10 Hz 的发布频率

# 确认飞控的 MAVLink 系统 ID 匹配
# 默认飞控 SYSID = 1，MAVROS 发送到 sysid=1, compid=1
# 如果改了飞控 SYSID，需要在 MAVROS launch 中设置 target_system_id
```

### 7.5 常见调试命令速查

```bash
# 话题列表
ros2 topic list | grep -E "mavros|joy"

# 实时观察 PWM override 输出
ros2 topic echo /mavros/rc/override

# 查看 IMU 数据
ros2 topic echo /mavros/imu/data --once

# 查看节点日志
ros2 run rov_pixhawk_control rov_pixhawk_control_node 2>&1 | tee node.log

# 查看飞控状态
ros2 topic echo /mavros/state --once

# 手动 arm 飞控（谨慎使用！）
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# 手动发送 RC override（紧急测试）
ros2 topic pub --once /mavros/rc/override mavros_msgs/msg/OverrideRCIn "
channels: [65535,65535,65535,65535,1500,1500,1500,1500,1500,1500,65535,65535,65535,65535,65535,65535,65535,65535]"
```

### 7.6 Pixhawk 不输出 PWM 信号

这是最常见的硬件问题之一。按以下步骤排查：

1. **确认飞控固件类型**：PX4 和 ArduPilot 的 SERVO 参数体系不同
2. **PX4**：检查 `SERVO_MAIN_FUNCTION` 参数，确保 MAIN 1~9 设为 `Motor 1`~`Motor 9`（或其他合适的输出功能）
3. **ArduPilot**：设置 `SERVO1_FUNCTION` ~ `SERVO9_FUNCTION` 为对应的电机/推进器功能
4. **安全开关**：部分 Pixhawk 有硬件安全开关（红色 LED 闪烁表示未解锁），需要按一下
5. **USB 供电模式**：仅 USB 供电时部分飞控的 PWM 输出口可能没有 5V 电源，需要接电池

---

## 附录 A：文件结构

```
rov_pixhawk_control/
├── GUIDE.md                          # ← 本文件
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── rov_pixhawk_control
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── rov_pixhawk_control/
    ├── __init__.py
    ├── utils.py                       # 共享工具函数（死区、PWM转换、四元数→欧拉角）
    ├── rov_pixhawk_control.py         # 主控制节点（推进器 + 模式切换 + 灯/舵机）
    └── attitude_publisher.py          # 姿态上报节点（ROS2 + UDP + CSV）
```

## 附录 B：CSV 日志格式

### B.1 推进器控制节点日志

节点启动后会在当前目录生成 `rov_imu_log_YYYYMMDD_HHMMSS.csv`：

| Timestamp | Roll(deg) | Pitch(deg) | Yaw(deg) | Mode |
|-----------|-----------|------------|----------|------|
| 1712345678 | -2.51 | 1.34 | 45.67 | 0 |
| 1712345688 | -2.48 | 1.31 | 45.71 | 1 |

> `Mode` 列为当前操作模式（0=手动, 1=辅助设备, 2=定姿, 3=定深）。
> 默认每 10 帧记录一次（`cfg_log_interval = 10`），约每秒记录一次（取决于 IMU 数据频率）。

### B.2 姿态上报节点日志

由 `attitude_publisher` 节点生成 `rov_attitude_log_YYYYMMDD_HHMMSS.csv`：

| Timestamp | Roll(deg) | Pitch(deg) | Yaw(deg) | Roll(rad) | Pitch(rad) | Yaw(rad) |
|-----------|-----------|------------|----------|-----------|------------|----------|
| 1712345678.123 | -2.51 | 1.34 | 45.67 | -0.0438 | 0.0234 | 0.7971 |

> 同时包含角度和弧度，方便不同场景使用。发布频率 50Hz，CSV 写入频率可配置（默认每 10 帧一行 ≈ 5Hz）。
