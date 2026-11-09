# reBot Teleoperation & Vision

reBot Teleoperation transforms Seeed reCamera SG2002 into an integrated Embodied AI and Robotic Arm controller.

## Features
- **Auto Wi-Fi AP Hotspot**: Broadcasts `reCamera-Arm-Teleop` AP (`192.168.42.1`) for zero-router phone connection.
- **Built-in vision (rebot-vision)**: H.264 video, YOLO object detection on the SG2002 TPU and ArUco markers, modes Off / YOLO / ArUco / Both chosen in the cockpit. No other camera app is needed; activating another camera app in the console stops this app.
- **YOLO model**: not included in the package; the model already on the device is used (`/userdata/rebot-teleop/models/yolo11n.cvimodel`, else the console's or yolo-detector's yolo11n). Without a model, ArUco still works.
- **Multi-Arm Driver Support**: SO-ARM 101 serial bus servos, reBot Arm RS (RobStride CAN), and reBot DM (Damiao CAN/UART).
- **Mobile Touch Teleoperation**: Multi-touch dual virtual joysticks, 2D kinematic pose visualizer, and keyboard hotkeys.
- **HuggingFace / LeRobot Dataset Recording**: Records synchronized joint observations and actions directly into Parquet datasets.
