````markdown
# UAV Competition Framework – Quadrotor Recon & Bombing

四旋翼无人机 **侦察 + 投弹比赛** 的 Python 程序框架。

目标设计：

- **模块化**：视觉、任务逻辑、OSD、MAVLink 互相解耦  
- **易调试**：每个模块都有自己的“小模拟程序”可以单独跑  
- **易扩展**：以后想换目标检测算法 / 飞控 / 控制策略，只改对应模块

当前仓库内可以先用一个 **假飞控 + 假视觉 + 状态机** 的 Demo 在电脑上跑起来（HUD 已移除，后续重写），再逐步替换成真实 MAVLink 和真实相机。  

---

## 1. 目录结构

> 你可以用之前的 `init_uav_competition.bat` 一键创建基础目录，然后把代码往里面填。

推荐结构（目标结构）：

```text
uav_competition/
├─ main.py                 # 主入口（组合所有黑盒，真实运行 / 假模拟都从这里进）
├─ config.yaml             # 参数配置：循环频率、高度、云台、RC 开关、投弹舵机等
├─ app/
│  ├─ __init__.py
│  ├─ config.py            # 读取 YAML 配置，转成 AppConfig
│  ├─ logging_config.py    # 统一日志配置
│  ├─ core/                # 公共数据结构 & 上下文
│  │  ├─ __init__.py
│  │  ├─ app_context.py    # AppContext(config + logger)
│  │  └─ models.py         # TelemetrySnapshot / VisionResult / ControlCommand
│  ├─ io/                  # 全部通过 MAVLink 与飞控交互
│  │  ├─ __init__.py
│  │  ├─ fc_client.py      # FcClient：MAVLink 封装（连接/心跳/发命令/读消息）
│  │  ├─ sensors.py        # Sensors：从 MAVLink 消息拼 TelemetrySnapshot
│  │  ├─ rc_input.py       # RcInput：解析 RC 通道 → 模式 / 投弹确认 等逻辑开关
│  │  ├─ gimbal_controller.py   # GimbalController：通过 MAVLink 控制云台舵机
│  │  └─ dropper_controller.py  # DropperController：通过 MAVLink 控制投弹舵机
│  ├─ vision/              # 视觉系统
│  │  ├─ __init__.py
│  │  └─ system.py         # VisionSystem：摄像头 + 目标检测 + 跟踪 → VisionResult + frame
│  ├─ mission/             # 任务状态机 & 控制逻辑
│  │  ├─ __init__.py
│  │  ├─ align_controller.py   # AlignController：dx/dy → yaw / gimbal / vx,vy
│  │  └─ mission_manager.py    # MissionManager：SEARCH / ALIGN / DROP 等状态机
│  └─ telemetry/           # 本地日志 / 远程传输
│     ├─ __init__.py
│     └─ logger.py         # TelemetryLogger：写 CSV / 记录关键数据
└─ sims/                   # 各黑盒的独立模拟程序
   ├─ sim_vision.py        # 只跑视觉模块：离线视频 / 图片测试识别 & dx/dy
   └─ sim_mission.py       # 只跑任务状态机：假 Telemetry + 假 VisionResult 看状态切换
````

> 目前你已经有一个单文件版 `main.py` Demo，可以先用它跑起来，之后按上面的结构拆文件。

---

## 2. 模块划分（“小黑盒”设计）

整个程序被拆成几个独立的“黑盒”，只通过少数数据结构互相通信，这样改其中一个不会影响其他模块。

### 2.1 Core 核心层

**文件：**

* `app/core/models.py`
* `app/core/app_context.py`
* `app/config.py`
* `app/logging_config.py`

**职责：**

* 定义几个 **全局通用的数据结构**（dataclass）：

  ```python
  @dataclass
  class TelemetrySnapshot:
      time: float
      mode: str
      armed: bool
      voltage: float
      alt: float
      vx: float
      vy: float
      yaw: float
      lat: float
      lon: float
      sats: int

  @dataclass
  class VisionResult:
      has_target: bool
      dx: float      # 像素坐标：目标相对画面中心偏移
      dy: float
      confidence: float

  @dataclass
  class ControlCommand:
      vx: float
      vy: float
      vz: float
      yaw_rate: float
      gimbal_pitch: float | None
      gimbal_yaw: float | None
      drop: bool
  ```

* 用 `AppConfig` 管理配置（从 `config.yaml` 读取）；

* 用 `AppContext` 和 `setup_logging()` 管理 logger 和 config。

所有黑盒都只用这些数据结构，不直接互相乱 import 内部细节。

---

### 2.2 IO / 飞控黑盒（MAVLink）

**文件：**

* `app/io/fc_client.py`
* `app/io/sensors.py`
* `app/io/rc_input.py`
* `app/io/gimbal_controller.py`
* `app/io/dropper_controller.py`

**硬件实际连接：**

> 舵机、接收机、GPS、电池传感器都接在飞控上，OrangePi 只需要一根 MAVLink 线。

**职责：**

* `FcClient`

  * 连接飞控（串口 / UDP）
  * 发送控制指令：速度、偏航、舵机、云台
  * 读 MAVLink 消息：姿态、位置、电池状态、RC 通道等

* `Sensors`

  * 从 `FcClient` 拿原始 MSG，拼成 `TelemetrySnapshot`

* `RcInput`

  * 把 RC 通道 PWM → 逻辑开关，比如：

    * mode: MANUAL / SEMI / AUTO
    * drop: OFF / ON

* `GimbalController`

  * 基于 MAVLink 的 MOUNT / SERVO 堆栈，提供：

    * `set_angles(pitch_deg, yaw_deg)`
    * `nudge(d_pitch, d_yaw)`
    * `set_scan_pose()`（高空斜视搜索姿态）

* `DropperController`

  * 用 `MAV_CMD_DO_SET_SERVO` 控制投弹舵机：

    * `drop_once()`：执行一次投弹动作（脉冲 → 回安全位）

**上层 Mission 完全不需要知道哪路舵机、哪路接收机，只跟这些高层接口交互。**

---

### 2.3 Vision 视觉黑盒

**文件：**

* `app/vision/system.py`

**职责：**

* 摄像头采集（真实版可能是 `/dev/videoX`）；
* 目标检测（颜色阈值 / 形状匹配 / YOLO / ArUco 等）；
* 目标跟踪 + 平滑，输出稳定的 dx/dy 和 has_target。

**统一接口：**

```python
class VisionSystem:
    def read(self) -> tuple[VisionResult, frame]:
        """
        ????????? + ?????????
        """
```

#### 视觉配置 & 使用说明（ArUco）

`config.yaml` → `vision` 字段：

- `device`: 摄像头索引或路径，例 `0` 或 `"/dev/video2"`；数字字符串在 `main.py` 中会被转成 int 传给 OpenCV。  
- `width` / `height`: 捕获分辨率，需设备支持；分辨率变化会影响内参与尺度。  
- `pixel_format`: 优先试 `MJPG`，不行再用 `YUYV`（会自动尝试 V4L2 / 默认 / GStreamer）。  
- `calibration_file`: OpenCV 标定文件路径（支持 `ost.yaml` 格式）；若提供且未手填矩阵，会自动加载到 `camera_matrix` / `dist_coeffs`。  
- `camera_matrix` / `dist_coeffs`: 可直接在 YAML 填入矩阵/畸变系数，优先级高于 `calibration_file`。示例矩阵格式：`[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]`；畸变示例 `[k1, k2, p1, p2, k3]`。  
- `aruco_dict`: 预置字典名，例 `DICT_4X4_50`。  
- `marker_length_m`: ArUco 标签实物边长（米），用于求姿态与坐标轴长度，必须与打印尺寸一致。

运行核对：

- `python3 sims/sim_vision.py` ? `python3 main.py`??? ArUco ???`sims/sim_vision.py` ??????? `dx/dy` ? `Tvec/Rvec`?`main.py` ??? headless?? HUD ????
- 如果显示缺失姿态：确认 OpenCV 带有 `aruco` 模块；未加载标定时会用默认内参，尺度和角度可能漂移。  
- 打不开摄像头时，可用 `v4l2-ctl --device=/dev/videoX --list-formats-ext` 查看支持的分辨率/格式，或检查用户是否在 `video` 组。

以后你从“颜色检测”换成“YOLO 模型”，不需要动 Mission，只要保持 `VisionResult` 不变即可。

---

### 2.4 Mission & Control 任务 / 控制黑盒

**文件：**

* `app/mission/align_controller.py`
* `app/mission/mission_manager.py`

**核心思想：**

> Mission 层只做“决策”：
> 输入 → Telemetry + VisionResult
> 输出 → ControlCommand
> 不直接发 MAVLink，由 IO 层执行。

**示例状态机设计（可扩展）：**

* `CLIMB_SCAN_ALT`：爬升到搜索高度（比如 30–50m）
* `SEARCH_SCAN`：高空斜视转圈 / 扫描
* `TARGET_LOCK`：发现目标，云台锁定 + 跟踪
* `APPROACH_TARGET`：接近目标，高度逐步下降
* `LOCAL_ALIGN`：低空精对准（目标居中）
* `READY_TO_DROP`：高度、对准、速度均满足
* `DROP`：触发投弹
* `EXIT`：拉升 / 退出区域

**AlignController：**

* 远距离对准：优先用云台 yaw/pitch + 小 yaw_rate 调整；
* 近距离对准：用 vx/vy 微调，让目标在机体正下方；
* 提供 `is_aligned(dx, dy)` 来判断是否可投。

**MissionManager：**

* 内部维护当前状态 + 切换条件；
* 对外只提供：
  `ControlCommand, mission_state = update(telemetry, vision)`。

---

### 2.5 HUD / OSD

当前 HUD 已在 `main.py` 实现：左侧相机画面，右侧侧边栏 OSD。OSD 包含状态机状态、dx/dy、置信度、对齐标记、tvec，以及飞控/RC 信息（电压、卫星数、模式、航向、速度、经纬度、armed 标记）。输出管道为 `appsrc ... fbdevsink device=/dev/fb0`，默认 `USE_FB=1`；`FBDEV` 可重定向设备，设 `USE_FB=0` 可关闭输出。

---
### 2.6 Telemetry / Logging 黑盒（可选但非常有用）

**文件：**

* `app/telemetry/logger.py`

**职责：**

* 按时间记录：

  * Telemetry
  * VisionResult（dx/dy、has_target）
  * ControlCommand（yaw_rate、vx、投弹动作）
* 写入 CSV，方便赛后分析 / 调参 / 回放。

---

### 2.7 Sims 模拟程序（每个黑盒都有一个小 demo）

**目录：**

* `sims/sim_vision.py`
  用现成视频 / 图片测试识别 & dx/dy。

* `sims/sim_mission.py`
  伪造 Telemetry & VisionResult，看状态机是否按预期切换、ControlCommand 是否合理。

## 3. 当前 Demo 版本（单文件 main.py）

为了方便起步，项目提供了一份**单文件 Demo**，只依赖：

```bash
pip install opencv-python numpy
```

功能（当前真机流）：

* `FakeFcClient`：假飞控，生成高度、电压、yaw、模式、速度、卫星数等假数据；
* `VisionSystem`：按 `config.yaml` 打开 `/dev/video1`（640x480@30，V4L2）做 ArUco 检测，输出 dx/dy/conf/tvec；
* HUD：左侧相机画面 + 右侧 OSD 侧边栏（状态机、dx/dy/conf、对齐标记、tvec、电压、卫星数、模式、航向、速度、经纬度、armed），通过 GStreamer 输出到 `/dev/fb0`；`USE_FB=0` 可关闭；
* `AlignController`：根据 dx/dy 和 tvec 做简单对准；
* `MissionManager`：简化状态机 `IDLE -> SEARCH -> ALIGN -> READY_TO_DROP -> DROP -> EXIT`
### 运行步骤

1. 安装依赖：

   ```bash
   pip install opencv-python numpy
   ```

2. 运行 Demo：

   ```bash
   python main.py
   ```

3. 看到一个窗口：

   * 背景为暗色画面；
   * 中心有绿色十字准心；
   * 偶尔出现红色圆圈（假目标），慢慢往中心靠近；
   * 左上角显示状态：SEARCH / ALIGN / READY_TO_DROP / DROP / EXIT 等；
   * 按 `ESC` 退出。


---

## 4. 从 Demo 过渡到真实无人机

推荐的开发顺序（避免一上来就真机踩坑）：

1. **先保持 Demo 运行**
   确保 `main.py` 的假模拟逻辑完全理解清楚。

2. **分别拆出模块：**

   * 把 `FakeFcClient` 抽出去变成 `app/io/fc_client.py`；
   * 把 `FakeVisionSystem` 抽出去变成 `app/vision/system.py`；
   * 把 `MissionManager` / `AlignController` 抽出去放 `app/mission/`；

3. **接入真实 MAVLink：**

   * 在 `FcClient` 里用 `pymavlink` 连接飞控；
   * 实现：

     * `read_telemetry_raw()`
     * `read_rc_channels()`
     * `send_velocity_body()`
     * `send_yaw_rate()`
     * `send_gimbal_angles()`
     * `send_servo()`；

4. **接入真实视觉：**

   * 在 `VisionSystem` 里打开 `/dev/videoX`；
   * 先用简单的颜色检测 / 形状检测验证 dx/dy；
   * 配置摄像头：`config.yaml` → `vision`
     * `device`: 节点或索引，例：`"/dev/video2"` 或 `0`
     * `width` / `height`: 例：`640` / `480`（与设备支持的分辨率匹配）
     * `pixel_format`: 先试 `MJPG`，不行再改为 `YUYV`
   * 打不开时快速排查：
     * `v4l2-ctl --device=/dev/videoX --list-formats-ext` 查看支持的分辨率/格式
     * 确认当前用户在 `video` 组（`groups`），否则 `sudo usermod -aG video $USER` 后重新登录

5. **调试任务状态机：**

   * 用仿真数据 & 真数据跑 `MissionManager`；
   * 逐步完善：

     * 高空斜视搜索逻辑；
     * 接近目标时的速度 / 高度控制；
     * 最终对准 + 投弹条件。

6. **真机外场试飞：**

   * 再逐步打开低增益 P 控制，观察行为；
   * 最后打开全流程自动对准+投弹。

---

## 5. TODO / Roadmap（可按你实际进度更新）

* [ ] 把 Demo 中的类拆分到 `app/` 结构
* [ ] 实现 `FcClient` 的真实 MAVLink 连接（pymavlink / mavsdk）
* [ ] 实现真实的 `VisionSystem`（摄像头 + 识别算法）
* [ ] 补充 `sims/` 各模块的独立模拟脚本
* [ ] 设计完整的搜索模式（高空斜视转圈 / 扫描）
* [ ] 调整 AlignController 参数（dx/dy 对应 yaw / gimbal / vx,vy）
* [ ] 完成一次安全、可控的外场自动对准 + 投弹测试

---

## 6. 许可 & 备注

* 本项目主要用于个人比赛 / 学习 / 实验；
* 你可以根据自己的实际飞控、传感器、云台设计，修改任何模块；
* 建议在任何自动控制逻辑启用前，**确保有可靠的手动 / 急停机制**（遥控优先、飞控模式切换等）。

---

## 7. OrangePi（Ubuntu 22.04）快速部署

OrangePi 的官方源通常较慢，建议先换成就近镜像，再安装 Python 环境与视频相关依赖。

1) 换源（示例使用清华 TUNA，按需换成你附近的镜像站；执行前备份原源）：  

```bash
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak2

sudo tee /etc/apt/sources.list <<EOF
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy-updates main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy-backports main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy-security main restricted universe multiverse

deb-src http://mirrors.aliyun.com/ubuntu-ports/ jammy main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu-ports/ jammy-updates main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu-ports/ jammy-backports main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu-ports/ jammy-security main restricted universe multiverse
EOF

sudo apt update
```

2) 安装基础依赖（Python/视频工具/编译工具）：  

```bash
sudo apt install -y python3 python3-pip python3-venv python3-opencv v4l-utils ffmpeg git build-essential
python3 -m pip config set global.index-url https://mirrors.aliyun.com/pypi/simple
python3 -m pip install --upgrade pip

sudo apt update

sudo apt install \
  gstreamer1.0-tools \
  gstreamer1.0-libav \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  v4l-utils

sudo apt install -y xserver-xorg xinit openbox xterm
```

> 如需使用 pip 版本的 OpenCV，请在虚拟环境中运行 `pip install opencv-python numpy`；如果已装 `python3-opencv` 可跳过。

3) （可选）创建虚拟环境并安装本项目依赖：  

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install opencv-python numpy
```

4) 摄像头权限检查：确保当前用户在 `video` 组（`groups`）；若不在，执行 `sudo usermod -aG video $USER` 后重新登录。

ls -l /dev/video*
v4l2-ctl --list-devices
ffplay /dev/video0


5) 运行 Demo 验证：  

```bash
python main.py
```

如果网络环境变化需要恢复官方源，使用备份文件还原 `/etc/apt/sources.list`，再执行 `sudo apt update`。
