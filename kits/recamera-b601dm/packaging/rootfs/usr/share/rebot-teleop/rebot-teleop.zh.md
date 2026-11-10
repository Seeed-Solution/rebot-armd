# reBot 具身视觉遥控与机械臂驱动

将 Seeed reCamera SG2002 打造为集机器视觉与机械臂驱动于一体的具身智能主控终端。

## 核心特性
- **开机自动开启 Wi-Fi 热点**：自动广播 `reCamera-Arm-Teleop`（IP `192.168.42.1`），手机免配网、免路由器直接操控。
- **内置视觉（rebot-vision）**：H.264 视频、SG2002 TPU 运行 YOLO 物体识别、ArUco 标记识别，在驾驶舱中选择 关 / YOLO / ArUco / 两者。无需其他相机应用；在控制台启用其他相机应用会停止本应用。
- **YOLO 模型**：安装包不含模型，使用设备上已有的模型（`/userdata/rebot-teleop/models/yolo11n.cvimodel`，否则使用控制台或 yolo-detector 自带的 yolo11n）。没有模型时 ArUco 仍可使用。
- **reBot B601-DM 机械臂驱动**：达妙电机，经 B4 底板 CAN 总线。
- **手机触控 Web 驾驶舱**：多点触控双虚拟摇杆、2D 机械臂正向运动学姿态图、长按防误触使能、E-STOP 一键急停。
- **重力补偿拖动示教与录制**：以 50Hz 频率采集关节角度与指令，每个片段导出为一个普通 Parquet 文件（不是 LeRobot 数据集）。
