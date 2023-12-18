# 能量机关检测模块
## 介绍
北极熊的能量机关检测模块，用于实现对能量机关靶标的识别、跟踪和预测。

<!-- ![pb_logo](../../docs/pb_logo.png) -->

<img src="../../docs/pb_logo.png" alt="WarehouseNetworkDiagram_1" width="200"/>

该项目为 PB_RM_Vision 的子模块。

作者: 小企鹅

运行环境：Ubuntu 22.04 / ROS2 Humble (未在其他环境下测试)

## 基本逻辑

~~~mermaid
graph TB
    A(接收图像)
    B(转换为灰度图后进行二值化处理)
    C(识别图中的R标)
    D(对R标进行PnP解算得到与能量机关间的距离)
    E(根据距离创建旋转圆环mask提取出靶标区域的二值图)
    F(识别图中的的靶标，并与旋转中心连线求出倾斜角度)
    G(合并相差不大的角度得到总角度数即靶标数)
    H(与上一次的数量作比较判断新增靶标)
    I(计算当前旋转速度)
    J(滤波得到实际速度)
    K(预测t时间后旋转到的位置)

    A --> B
    B --> C
    C --> D
    D --> E
    E --> F
    F --> G
    G --> H
    H --> I
    I --> J
    J --> K
~~~
