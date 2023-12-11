# 北极熊视觉系统

> 本项目为北极熊视觉系统，基于ROS2实现不同兵种所需的视觉功能。

日前迁移了rm_vision进入系统，实现了装甲板瞄准功能，现在正在开发能量机关识别以及矿物拾取和兑换识别。

## 开始使用

clone 本项目后首先在根目录下打开命令行编译项目：
```shell
colcon build --symlink-install
```


启动所有模块
- <details>
    <summary>rm_vision</summary>

    仅包括开源装甲板识别模块

    ```shell
    sudo chmod 777 /dev/ttyACM0

    source install/setup.bash
    ros2 launch rm_vision_bringup vision_bringup.launch.py
    ```

  </details>

- 步兵

  包括`装甲板识别`和`能量机关识别`模块

  ```shell
  sudo chmod 777 /dev/ttyACM0

  source install/setup.bash
  ros2 launch rm_vision_bringup infantry_bringup.launch.py
  ```

- 英雄

  包括`装甲板识别`模块

  ```shell
  sudo chmod 777 /dev/ttyACM0

  source install/setup.bash
  ros2 launch rm_vision_bringup hero_bringup.launch.py
  ```

- 工程

  包括`兑换站识别`和`矿石识别`模块


- 哨兵

  包括`装甲板识别`模块



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

## 相关信息
### 通讯协议
- miniPC 接收的信息结构如下
  ```C++
  struct ReceivePacket
  {
    uint8_t header = 0x5A;
    uint8_t detect_color : 1;  // 0-red 1-blue
    bool reset_tracker : 1;
    uint8_t reserved : 6;
    float roll;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum = 0;
  } __attribute__((packed));
  ```

- miniPC 发送的信息结构如下
  ```C++
  struct SendPacket
  {
    uint8_t header = 0xA5;
    bool tracking : 1;
    uint8_t id : 3;          // 0-outpost 6-guard 7-base
    uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
    uint8_t reserved : 1;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    uint16_t checksum = 0;
  } __attribute__((packed));
  ```

### 颜色定义
- `0` - `red`
- `1` - `blue`

## 其他文档
rm_vision 部署文档： [部署华师视觉项目](https://flowus.cn/lihanchen/share/0d472992-f136-4e0e-856f-89328e99c684) \
测算相机畸变与内参矩阵：[相机标定](https://flowus.cn/lihanchen/share/02a518a0-f1bb-47a5-8313-55f75bab21b5)