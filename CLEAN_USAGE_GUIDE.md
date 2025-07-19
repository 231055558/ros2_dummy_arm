# Dummy机械臂系统 - 简化使用指南

## 🚀 快速启动

### 1. 构建系统
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. 启动真实机器人控制器
```bash
# 终端1：硬件控制器
ros2 run dummy_controller dummy_arm_controller
```

### 3. 启动MoveIt + RViz演示
```bash
# 终端2：完整演示环境
ros2 launch dummy_moveit_config demo_real.launch.py

# 或使用简化版本
ros2 launch dummy_moveit_config demo_real_simple.launch.py
```

### 4. 启动Gazebo同步 (可选)
```bash
# 终端3：Gazebo仿真同步
ros2 launch dummy_moveit_config gazebo_sync.launch.py
```

## 🔧 系统测试

### 测试关节状态发布
```bash
# 测试关节状态
python3 src/dummy_moveit_config/joint_state_test.py

# 手动检查
ros2 topic echo /joint_states
```

### 测试Gazebo同步
```bash
# 测试Gazebo同步功能
python3 src/dummy_moveit_config/test_gazebo.py

# 检查Gazebo关节状态
ros2 topic echo /gazebo_joint_states
```

### 测试机械臂运动
```bash
# 运行综合测试
ros2 run dummy_controller test_trajectory

# 手动运动命令
ros2 action send_goal /dummy_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  points:
  - positions: [0.5, -0.5, 0.8, 0.0, 0.0, 0.0]
    time_from_start: {sec: 3}
"
```

## 📋 核心功能

### RViz交互式控制
1. 在RViz中拖动机械臂到目标位置
2. 点击"Plan & Execute"执行运动
3. 观察真实机械臂跟随运动

### Gazebo仿真同步
1. 启动Gazebo同步后，Gazebo中的机械臂会实时跟随真实机械臂
2. Gazebo订阅/joint_states话题获取真实机械臂位置
3. 支持可视化验证和调试

### 编程接口
```python
# 使用pymoveit2库
from pymoveit2 import MoveIt2
moveit2 = MoveIt2(node, joint_names, base_link, end_effector, group_name)
moveit2.move_to_configuration([0.0, -0.5, 0.8, 0.0, 0.0, 0.0])
```

## 🔍 故障排除

### 如果RViz显示不同步
1. 确保dummy_arm_controller正在运行
2. 检查关节状态：`ros2 topic echo /joint_states`
3. 重启系统按正确顺序

### 如果Gazebo不同步
1. 确保gazebo_sync节点正在运行：`ros2 node list | grep gazebo`
2. 检查Gazebo关节状态：`ros2 topic echo /gazebo_joint_states`
3. 运行测试脚本：`python3 src/dummy_moveit_config/test_gazebo.py`

### 如果无法连接机器人
1. 检查USB连接：`lsusb`
2. 检查串口：`ls /dev/ttyUSB* /dev/ttyACM*`
3. 重新插拔USB连接

## 📚 系统架构

### 核心组件
- **dummy_arm_controller**: 硬件接口和关节状态发布
- **robot_state_publisher**: TF变换发布
- **move_group**: MoveIt运动规划节点
- **rviz2**: 3D可视化和交互界面

### Gazebo组件
- **gazebo_sync**: 同步节点，将真实机械臂状态传递给Gazebo
- **gazebo_robot_state_publisher**: Gazebo中的机器人状态发布器
- **Gazebo仿真环境**: 3D物理仿真

## 🔄 话题结构

### 主要话题
- `/joint_states`: 真实机械臂关节状态 (10Hz)
- `/robot_description`: 机器人URDF模型
- `/tf`: 坐标变换信息

### Gazebo话题
- `/gazebo_joint_states`: Gazebo机械臂关节状态
- `/gazebo/{joint}_position_controller/command`: Gazebo关节位置命令

## ✅ 成功标志

### 基本系统
- ✅ 关节状态以约10Hz频率发布
- ✅ RViz中虚拟机械臂与真实机械臂位置同步
- ✅ 运动规划和执行功能正常
- ✅ 无关节状态相关错误消息

### Gazebo同步
- ✅ Gazebo中机械臂与真实机械臂位置一致
- ✅ gazebo_sync节点正常运行
- ✅ /gazebo_joint_states话题正常发布

## 🎯 注意事项

- 始终先启动dummy_arm_controller
- 确保硬件正确连接后再启动MoveIt
- Gazebo同步为可选功能，用于可视化和调试
- 使用简化版本启动文件获得更好的稳定性 