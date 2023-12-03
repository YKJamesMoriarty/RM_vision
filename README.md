# 北极熊视觉系统
## 模块划分
目前初步将整体系统划分为以下几个模块，各模块分别成为一个节点，通过ROS2节点进行通信：

- [x] 单目相机模块
- [ ] 全景相机模块
- [x] 串口通信模块
- [x] 装甲板识别模块
- [ ] 能量机关识别模块
- [ ] 弹丸识别模块
- [ ] 矿石识别模块
- [ ] 兑换站识别模块
- [ ] 自车状态模块（整合到串口模块中）
- [ ] 目标状态估计模块
- [ ] 车体动作指令模块

## 开始使用

clone 本项目后首先在根目录下打开命令行构建项目：
```shell
colcon build --symlink-install
```


启动
```shell
sudo chmod 777 /dev/ttyACM0
source install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

可视化
```shell
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

