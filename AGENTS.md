# Repository Guidelines

Jetson Orin Nano 上运行的两套核心代码：`~/wheeltec_ros2`（ROS 2 Humble 控制与感知）与 `~/py-xiaozhi-main\ `（语音/MCP 客户端，目录名含尾随空格）。语音侧通过 MCP 工具调用 ROS 节点，实现语音 → 机器人动作的闭环。

## 项目结构与职责
- `wheeltec_ros2/src/`：底盘与传感器（`turn_on_wheeltec_robot`、`wheeltec_mic_ros2`、`wheeltec_lidar_ros2`、`wheeltec_imu`）、视觉跟随（`people_follow`、`ultralytics_ros2`）、导航（`navigation2-humble`、`nav2_waypoint_cycle`）、辅助工具（`wheeltec_robot_keyboard`、`web_video_server-ros2`）。
- `py-xiaozhi-main\ /src/`：`application.py` 管理插件；`mcp/tools/robot_controller` 用 `rclpy` 创建 `RobotRosNode` 并向 `/cmd_vel`、`/mode`、`/model`、`/motion_msg` 发布；`audio_processing/` 负责唤醒词与 VAD；`plugins/` 下为音频、MCP、UI 等插件。
- `build/ install/ log/` 为 ROS 构建产物；`msc/` 可放测试 bag 或调试资料。

## 核心命令
```bash
# ROS 环境与构建
cd ~/wheeltec_ros2 && source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# 典型运行
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
ros2 launch wheeltec_mic_ros2 base.launch.py            # 底盘+语音+雷达+摄像头+跟随
python main.py --mode cli --protocol websocket          # 需在已 source ROS 的 Shell 中
```

## ROS ⇆ 语音桥接（关键话题）
- `/cmd_vel` (`geometry_msgs/Twist`): 底盘速度，turn_on_wheeltec_robot 订阅；MCP 的 move/turn/stop 工具通过 `robot_controller.publish_velocity` 写入。
- `/mode` (`std_msgs/Int8`): YOLO 人体跟随模式（1=休眠，2=跟随），由 MCP `start_human_following/stop_human_following` 切换；people_follow、ultralytics_ros2 订阅。
- `/model` (`std_msgs/Int32`): 动作/表情组，FT/move_ft 订阅；MCP `actionGroup`、`greet`、情绪模式使用。
- `/motion_msg` (`wheeltec_mic_msg/MotionControl`): 声源追踪输入，wheeltec_mic_ros2 订阅；MCP `start_sound_following/stop_sound_following` 发布。

## 开发/调试提示
- 保证只存在一个 `/cmd_vel` 主发布者；用 `ros2 topic info /cmd_vel` 查重。
- 如果 `rclpy`/自定义消息未找到，确认已 `source /opt/ros/humble/setup.bash` 和 `install/setup.bash`，并在与 ROS 相同 Python 环境运行 `py-xiaozhi`。
- 语音离线命令与短语在 `wheeltec_mic_ros2/config/call.bnf`、`src/command_recognition.cpp`；调整后需重建包。
- YOLO 叠加画面可在浏览器访问 `http://<机器人IP>:8080/stream?topic=/detected_image`，验证跟随状态。
- 启动失败先查 `/dev/wheeltec_controller`、`/dev/wheeltec_lidar`、`/dev/video*` 是否存在，确保 udev 权限与硬件连接正常。
- 扩展语音到动作的映射时，同时更新 `robot_controller/manager.py` 中的 MCP 工具描述，保持 LLM 行为一致。
- 导航或跟随跑偏时先记录 rosbag（/cmd_vel、/tf、/scan），并标注场景，便于离线复现解决。
- 提交前可在仿真或短距离场地先验证关键动作，减少上机调试时间和风险。

## 代码风格与提交
- C++：2 空格，`clang-format`（ROS/Google），头文件自包含；新增测试用 `ament_cmake_gtest`。Python：`black --line-length 100`、`ament_flake8`、`ament_pep257`。
- Commit 前缀包名，祈使语气，例如 `wheeltec_mic_ros2: add follow phrase`。PR 需写明硬件/launch 命令、测试结果（`colcon build/test`）、涉及的参数文件，并附必要截图/日志。
