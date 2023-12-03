# ROS2的使用
- [ROS2的使用](#ros2的使用)
- [新建软件包](#新建软件包)
- [编译项目](#编译项目)
- [报错](#报错)
  - [编辑器找不到ROS2头文件](#编辑器找不到ros2头文件)

# 新建软件包
在ROS2中，你可以使用 `ros2 pkg create` 命令来创建一个新的C++软件包。

如通过以下代码创建一个名为rm_rune_detector的新软件包的示例，该软件包依赖于rclcpp和std_msgs
```shell
ros2 pkg create --build-type ament_cmake rm_rune_detector --dependencies rclcpp std_msgs
```
这将在当前目录下创建一个名为`rm_rune_detector`的新目录，其中包含了一个基本的ROS2软件包结构，以及一个`CMakeLists.txt`文件，该文件已经配置好了对`rclcpp`和`std_msgs`的依赖。

你可以根据需要修改这个命令，将rm_rune_detector替换为你想要的软件包名，将rclcpp和std_msgs替换为你的软件包所依赖的其他ROS2软件包。

# 编译项目
使用 `colcon build` 来编译项目
> Tip:使用`--symlink-install`命令可以创建配置文件的链接，这样在更改配置文件后可以不用重新编译。

如使用以下命令编译项目

```shell
colcon build --symlink-install
```

# 报错
## 编辑器找不到ROS2头文件
有时候会发现如下的ROS头文件会被加上下划波浪线并显示找不到

```
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
```

点击 `快速修复` - `编辑includePath` 后找到 `包含路径`，并增加一行添加以下内容（其中humble为安装的ROS版本）

```
/opt/ros/humble/**
```
