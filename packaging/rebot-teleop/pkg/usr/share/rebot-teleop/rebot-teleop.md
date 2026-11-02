# reBot Teleoperation & Vision

reBot Teleoperation transforms Seeed reCamera SG2002 into an integrated Embodied AI and Robotic Arm controller.

## Features
- **Auto Wi-Fi AP Hotspot**: Broadcasts `reCamera-Arm-Teleop` AP (`192.168.42.1`) for zero-router phone connection.
- **Hardware TPU Acceleration**: Direct support for SG2002 TPU YOLOv11 target detection.
- **Multi-Arm Driver Support**: SO-ARM 101 serial bus servos, reBot Arm RS (RobStride CAN), and reBot DM (Damiao CAN/UART).
- **Mobile Touch Teleoperation**: Multi-touch dual virtual joysticks, 2D kinematic pose visualizer, and keyboard hotkeys.
- **HuggingFace / LeRobot Dataset Recording**: Records synchronized joint observations and actions directly into Parquet datasets.
