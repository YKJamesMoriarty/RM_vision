# 北极熊视觉系统

> 本项目为北极熊视觉系统，基于ROS2实现不同兵种所需的视觉功能。

日前迁移了rm_vision进入系统，实现了装甲板瞄准功能，现在正在开发能量机关识别以及矿物拾取和兑换识别。

<details>
  <summary>模块划分</summary>
  &nbsp;
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

</details>

## 开始使用

clone 本项目后首先在根目录下打开命令行编译项目：
```shell
colcon build --symlink-install
```


启动所有模块
- rm_vision
  ```shell
  sudo chmod 777 /dev/ttyACM0

  source install/setup.bash
  ros2 launch rm_vision_bringup vision_bringup.launch.py
  ```
- 步兵
  ```shell
  sudo chmod 777 /dev/ttyACM0

  source install/setup.bash
  ros2 launch rm_vision_bringup infantry_bringup.launch.py
  ```
- 英雄
  ```shell
  sudo chmod 777 /dev/ttyACM0

  source install/setup.bash
  ros2 launch rm_vision_bringup hero_bringup.launch.py
  ```
- 工程
- 哨兵

启动可视化
```shell
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

<details>
  <summary>单独运行子模块</summary>
  &nbsp;
一般用不上，写在这只为了有时开发要调用 rv 独立模块调试

- 自瞄模块
    ```Shell
    source install/setup.bash
    ros2 launch auto_aim_bringup auto_aim.launch.py 
    ```

- 海康相机模块
    ```Shell
    source install/setup.bash
    ros2 launch hik_camera hik_camera.launch.py
    ```

- 串口模块
    ```Shell
    sudo chmod 777 /dev/ttyACM0

    source install/setup.bash
    ros2 launch rm_serial_driver serial_driver.launch.py
    ```

- 能量机关识别模块
    ```Shell
    source install/setup.bash
    ros2 launch rm_rune_detector rm_rune_detector.launch.py
    ```

</details>

## 其他文档
rm_vision 部署文档： [部署华师视觉项目](https://flowus.cn/lihanchen/share/0d472992-f136-4e0e-856f-89328e99c684) \
测算相机畸变与内参矩阵：[相机标定](https://flowus.cn/lihanchen/share/02a518a0-f1bb-47a5-8313-55f75bab21b5)