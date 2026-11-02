# reBot 具身视觉遥控与机械臂驱动

将 Seeed reCamera SG2002 打造为集机器视觉与机械臂驱动于一体的具身智能主控终端。

## 核心特性
- **开机自动开启 Wi-Fi 热点**：自动广播 `reCamera-Arm-Teleop`（IP `192.168.42.1`），手机免配网、免路由器直接操控。
- **硬件 TPU 加速**：原生集成 SG2002 硬件 TPU YOLOv11 目标检测。
- **多型号机械臂驱动**：支持 SO-ARM 101 串口总线舵机、reBot Arm RS（RobStride CAN）以及 reBot DM（达妙双模）。
- **手机触控 Web 驾驶舱**：多点触控双虚拟摇杆、2D 机械臂正向运动学姿态图、长按防误触使能、E-STOP 一键急停。
- **LeRobot 数据集录制**：以 50Hz 频率实时采集关节角度与指令，一键导出标准 HuggingFace Parquet 格式。
