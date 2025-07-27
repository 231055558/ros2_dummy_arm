# Gazebo 和 RViz 同步演示

这个包演示了如何在 Gazebo 仿真环境和 RViz 可视化工具之间同步物体状态。

## 功能特点

- **Gazebo 仿真**：包含一个可移动的红色方块
- **RViz 可视化**：实时显示方块的位置和坐标信息  
- **自动控制**：方块可以按照不同模式自动移动
- **实时同步**：Gazebo 中的方块状态实时同步到 RViz

## 文件结构

```
gazebo_rviz_sync_demo/
├── gazebo_rviz_sync_demo/
│   ├── __init__.py
│   ├── cube_synchronizer.py    # 同步节点
│   └── cube_controller.py      # 控制节点
├── worlds/
│   └── simple_cube.world       # Gazebo 世界文件
├── launch/
│   └── sync_demo.launch.py     # 主启动文件
├── rviz/
│   └── cube_sync.rviz          # RViz 配置
└── README.md
```

## 安装和编译

1. 确保在 ROS2 工作空间中：
```bash
cd ~/ros2_ws/src
```

2. 编译包：
```bash
cd ~/ros2_ws
colcon build --packages-select gazebo_rviz_sync_demo
source install/setup.bash
```

## 使用方法

### 基本启动

启动完整演示（包含自动移动的方块）：
```bash
ros2 launch gazebo_rviz_sync_demo sync_demo.launch.py
```

### 启动参数

- `auto_move` (默认: true): 是否自动移动方块
- `movement_mode` (默认: circle): 移动模式
  - `circle`: 圆形轨迹
  - `square`: 正方形轨迹  
  - `line`: 直线往返
  - `stop`: 停止移动

示例：
```bash
# 正方形移动模式
ros2 launch gazebo_rviz_sync_demo sync_demo.launch.py movement_mode:=square

# 仅启动同步，不自动移动
ros2 launch gazebo_rviz_sync_demo sync_demo.launch.py auto_move:=false
```

### 手动控制方块

如果禁用了自动移动，可以手动控制方块：
```bash
ros2 topic pub /demo_cube/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 监控话题

查看相关话题：
```bash
# 查看方块位置信息
ros2 topic echo /demo_cube/odom

# 查看同步标记
ros2 topic echo /cube_markers

# 查看所有话题
ros2 topic list
```

## 工作原理

1. **Gazebo 仿真**：
   - 加载包含红色方块的世界文件
   - 方块配置了 `libgazebo_ros_planar_move.so` 插件
   - 发布里程计信息到 `/demo_cube/odom` 话题

2. **同步节点** (`cube_synchronizer`):
   - 订阅 Gazebo 的里程计话题
   - 将位置信息转换为 RViz 标记
   - 发布到 `/cube_markers` 话题

3. **RViz 可视化**：
   - 显示方块的 3D 标记
   - 显示实时坐标信息
   - 显示 TF 变换关系

4. **控制节点** (`cube_controller`，可选):
   - 生成不同模式的运动轨迹
   - 发布速度命令到 `/demo_cube/cmd_vel`

## 扩展用途

这个演示可以作为以下项目的基础：

- 机械臂避障系统
- 多机器人协调
- 传感器数据可视化
- 实时路径规划

## 故障排除

### Gazebo 启动失败
确保安装了 Gazebo 和相关插件：
```bash
sudo apt install gazebo11 gazebo11-plugins-ros
```

### RViz 无法显示标记
检查话题是否正常发布：
```bash
ros2 topic hz /cube_markers
```

### TF 变换问题
检查 TF 树：
```bash
ros2 run tf2_tools view_frames
``` 