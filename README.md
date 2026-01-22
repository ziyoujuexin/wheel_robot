# 轮式机器人

## SSH 

默认IP（WIFI连接`LIANQIU`）：`192.168.1.175/176`

User：`wheeltec`

passwd：`dongguan`

```bash
ssh wheeltec@192.168.1.175
```



## 一键启动
```bash
ros2 launch ros2 launch wheeltec_mic_ros2 base.launch.py
```

## 启动底盘
```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```