# Dummy机械臂快速启动指南

## 🚀 5分钟快速体验

### 1. 基础启动（必须先运行）

```bash
# 终端1：启动硬件控制器
cd ~/ros2_ws
source install/setup.bash
ros2 run dummy_controller dummy_arm_controller

# 终端2：启动MoveIt演示环境
cd ~/ros2_ws
source install/setup.bash
ros2 launch dummy_moveit_config demo.launch.py
```

### 2. 立即可用的演示命令

```bash
# 新终端：运行综合测试
ros2 run dummy_controller test_trajectory

# 关节空间运动示例
ros2 action send_goal /dummy_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  points:
  - positions: [0.5, -0.5, 0.8, 0.0, 0.0, 0.0]
    time_from_start: {sec: 3}
"

# 添加碰撞对象演示（使用简单版本，无需额外依赖）
ros2 run dummy_controller add_collision_object_simple
```

### 3. 最有趣的演示

**🎯 RViz交互式控制：**
1. 在RViz中拖动机械臂到新位置
2. 点击"Plan & Execute"
3. 观察真实机械臂跟随运动

**🤖 预设动作演示：**
```bash
# 挥手动作（在测试程序中）
ros2 run dummy_controller test_trajectory
# 选择挥手测试部分

# 平滑轨迹演示
# 机械臂将执行复杂的多点轨迹
```

## 📊 系统状态监控

```bash
# 实时查看关节状态
ros2 topic echo /joint_states

# 查看可用控制器
ros2 control list_controllers

# 检查系统话题
ros2 topic list
```

## 🎮 简单控制命令

**移动到常用位置：**
```bash
# 回到初始位置
ros2 action send_goal /dummy_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 3}
"

# 工作准备位置
ros2 action send_goal /dummy_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  points:
  - positions: [0.0, -1.3089, 1.5707, 0.0, 0.0, 0.0]
    time_from_start: {sec: 4}
"
```

## 🔧 如果遇到问题

**硬件控制器无法启动：**
- 检查USB连接
- 确认设备权限：`sudo chmod 666 /dev/ttyUSB*`

**MoveIt无法规划：**
- 确认目标位置在工作空间内
- 检查是否有碰撞

**Action连接失败：**
- 确认硬件控制器已启动
- 重启控制器：`ros2 control switch_controllers --start dummy_arm_controller`

## 🎯 接下来探索

1. **阅读完整文档：** `DUMMY_ARM_DOCUMENTATION.md`
2. **尝试Python编程：** 使用pymoveit2库
3. **自定义演示：** 修改测试程序
4. **添加功能：** 集成传感器或新的控制算法

**享受您的机械臂控制之旅！** 🤖✨ 