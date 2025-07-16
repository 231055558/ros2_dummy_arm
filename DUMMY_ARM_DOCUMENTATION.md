# Dummy机械臂ROS 2控制系统完整文档

## 系统概述

本系统是一个基于ROS 2的6自由度机械臂控制系统，包含物理机械臂硬件控制、MoveIt运动规划、RViz可视化以及多种编程接口。系统支持实时控制、轨迹规划、碰撞检测和交互式操作。

## 1. 机械臂信息访问

### 1.1 关节状态信息

**话题订阅：**
```bash
# 实时关节状态
ros2 topic echo /joint_states

# 关节名称
joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
```

**Python代码访问：**
```python
import rclpy
from sensor_msgs.msg import JointState

class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
    
    def joint_callback(self, msg):
        positions = list(msg.position)  # 当前关节角度(弧度)
        names = list(msg.name)         # 关节名称
```

### 1.2 机械臂物理参数

**关节限制范围：**
- Joint1: -180° 到 +180° (-3.14 to 3.14 rad)
- Joint2: -75° 到 +90° (-1.31 to 1.57 rad)
- Joint3: -90° 到 +90° (-1.57 to 1.57 rad)
- Joint4: -180° 到 +180° (-3.14 to 3.14 rad)
- Joint5: -90° 到 +90° (-1.57 to 1.57 rad)
- Joint6: -180° 到 +180° (-3.14 to 3.14 rad)

**运动参数：**
- 最大速度：3.15 rad/s
- 默认速度比例：10% (可在joint_limits.yaml修改)
- 默认加速度比例：10%

**坐标系信息：**
- 基坐标系：`base_link`
- 末端执行器：`link6_1_1`
- 运动组：`dummy_arm`

### 1.3 TF坐标变换

```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 实时查看变换
ros2 run tf2_ros tf2_echo base_link link6_1_1
```

## 2. 机械臂控制方法

### 2.1 关节空间控制

**方法一：直接发送轨迹命令**
```bash
# 移动到指定关节角度 (度)
ros2 action send_goal /dummy_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  points:
  - positions: [0.0, -0.5, 0.8, 0.0, 0.0, 0.0]
    time_from_start: {sec: 3}
"
```

**方法二：使用Python API**
```python
# 使用Action客户端
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

action_client = ActionClient(node, FollowJointTrajectory, 
                           'dummy_arm_controller/follow_joint_trajectory')

# 使用pymoveit2库
from pymoveit2 import MoveIt2
moveit2 = MoveIt2(node, joint_names, base_link, end_effector, group_name)
moveit2.move_to_configuration([0.0, -0.5, 0.8, 0.0, 0.0, 0.0])
```

**方法三：使用测试程序**
```bash
ros2 run dummy_controller test_trajectory
```

### 2.2 笛卡尔空间控制

**使用MoveIt规划器：**
```python
# 移动到指定位置和姿态
moveit2.move_to_pose(
    position=[0.3, 0.2, 0.4],           # 位置 (x, y, z)
    quat_xyzw=[0.0, 0.0, 0.0, 1.0],     # 四元数姿态
    cartesian=True                       # 笛卡尔路径规划
)
```

**使用内置moveit_server：**
```bash
ros2 run dummy_controller moveit_server
```

### 2.3 交互式控制

**RViz交互式控制：**
```bash
# 启动完整demo环境
ros2 launch dummy_moveit_config demo.launch.py

# 在RViz中：
# 1. 拖动机械臂到目标位置
# 2. 点击"Plan & Execute"执行运动
```

### 2.4 预定义位姿

**系统内置位姿：**
```python
# 在dummy-ros2.srdf中定义
- home: [0, 0, 0, 0, 0, 0]              # 初始位置
- reset: [0, -1.1276, 1.4052, 0, 0, 0] # 准备位置

# 在测试程序中的额外位姿
- ready: [0, -1.3089, 1.5707, 0, 0, 0] # 工作准备位置
- wave: [0, -0.5, 0.8, 0, 1.5, 0]      # 挥手位置
```

## 3. 仿真环境功能

### 3.1 MoveIt规划功能

- **运动规划算法：** RRT, RRTConnect, PRM等
- **碰撞检测：** 实时环境碰撞检测
- **路径优化：** 自动优化运动轨迹
- **逆运动学求解：** KDL求解器

### 3.2 可视化功能

**RViz插件包含：**
- 机械臂3D模型显示
- 运动规划轨迹预览
- 碰撞对象管理
- 交互式标记控制
- 关节状态实时显示

### 3.3 安全功能

- 关节限制检查
- 速度和加速度限制
- 碰撞避免
- 紧急停止功能

### 3.4 规划场景管理

```python
# 添加碰撞对象（推荐使用简单版本）
ros2 run dummy_controller add_collision_object_simple

# 如果有完整的moveit_commander依赖
ros2 run dummy_controller add_collision_object

# 清除规划场景
from pymoveit2 import MoveIt2
moveit2.clear_collision_objects()
```

## 4. 演示测试程序

### 4.1 基础功能测试

```bash
# 综合测试程序
ros2 run dummy_controller test_trajectory
```

**测试内容：**
- 基本关节运动测试
- 平滑轨迹运动测试
- 关节限制测试
- 挥手动作演示

### 4.2 MoveIt功能演示

```bash
# 启动完整演示环境
ros2 launch dummy_moveit_config demo.launch.py

# 单独启动move_group
ros2 launch dummy_moveit_config move_group.launch.py
```

### 4.3 PyMoveIt2示例

```bash
# 关节空间运动
ros2 run pymoveit2 ex_joint_goal --ros-args -p joint_positions:="[0.5, -0.5, 0.8, 0.0, 0.0, 0.0]"

# 笛卡尔空间运动
ros2 run pymoveit2 ex_pose_goal --ros-args -p position:="[0.3, 0.2, 0.4]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]"

# 正运动学测试
ros2 run pymoveit2 ex_fk

# 逆运动学测试
ros2 run pymoveit2 ex_ik
```

### 4.4 碰撞检测演示

```bash
# 添加碰撞对象
ros2 run pymoveit2 ex_collision_primitive

# 碰撞避让演示
ros2 run pymoveit2 ex_collision_custom
```

### 4.5 趣味演示

```bash
# 机械臂挥手
# 在test_trajectory中包含挥手演示

# 复杂轨迹演示
# 运行测试程序中的smooth_trajectory功能
```

## 5. 参数配置修改

### 5.1 运动参数调整

**文件位置：** `src/dummy_moveit_config/config/joint_limits.yaml`

```yaml
# 修改速度和加速度比例
default_velocity_scaling_factor: 0.1   # 速度比例 (0.1 = 10%)
default_acceleration_scaling_factor: 0.1  # 加速度比例

# 修改单个关节限制
joint_limits:
  joint1:
    max_velocity: 3.15      # 最大速度
    max_acceleration: 5.0   # 最大加速度
```

### 5.2 灯光控制参数

**在控制器代码中修改：** `src/dummy_controller/dummy_controller/dummy_arm_controller.py`

```python
# RGB灯光模式
self.my_driver.robot.set_rgb_mode(4)  # 4=绿色, 0=关闭, 1=红色, 2=蓝色, 3=黄色
```

### 5.3 控制器配置

**文件位置：** `src/dummy_moveit_config/config/moveit_controllers.yaml`

```yaml
moveit_simple_controller_manager:
  controller_names:
    - dummy_arm_controller
  
  dummy_arm_controller:
    action_ns: follow_joint_trajectory  # Action命名空间
    type: FollowJointTrajectory         # 控制器类型
```

### 5.4 运动学求解器配置

**文件位置：** `src/dummy_moveit_config/config/kinematics.yaml`

```yaml
dummy_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005  # 搜索精度
  kinematics_solver_timeout: 0.5              # 超时时间
```

### 5.5 规划器配置

**文件位置：** `src/dummy_moveit_config/config/ompl_planning.yaml`

```yaml
planner_configs:
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0  # 0表示使用默认值
  
  # 添加其他规划器配置
```

### 5.6 可视化配置

**RViz配置文件：** `src/dummy_moveit_config/config/moveit.rviz`

可以修改：
- 机械臂外观颜色
- 显示选项
- 视角设置
- 插件配置

## 6. 高级功能

### 6.1 路径约束

```python
# 设置路径约束
moveit2.set_path_position_constraint(
    position=[0.3, 0.0, 0.4],
    tolerance=0.1
)
```

### 6.2 速度和加速度控制

```python
# 动态调整运动参数
moveit2.max_velocity = 0.3       # 30%最大速度
moveit2.max_acceleration = 0.2   # 20%最大加速度
```

### 6.3 碰撞对象管理

```python
# 添加碰撞盒子
moveit2.add_collision_box(
    id="table",
    size=[1.0, 1.0, 0.1],
    position=[0.5, 0.0, 0.0]
)

# 移除碰撞对象
moveit2.remove_collision_object("table")
```

### 6.4 多点轨迹规划

```python
# 笛卡尔路径规划
waypoints = [
    [0.3, 0.3, 0.4],
    [0.3, -0.3, 0.4],
    [0.5, 0.0, 0.6]
]

for point in waypoints:
    moveit2.move_to_pose(position=point, cartesian=True)
```

## 7. 故障排除

### 7.1 常见问题

**Action服务器连接失败：**
```bash
# 检查控制器状态
ros2 control list_controllers

# 重启控制器
ros2 control switch_controllers --start dummy_arm_controller
```

**MoveIt规划失败：**
- 检查目标位置是否可达
- 确认关节限制设置
- 验证碰撞检测设置

**硬件连接问题：**
- 检查USB连接
- 验证设备权限
- 重启硬件控制器

### 7.2 调试工具

```bash
# 查看所有可用话题
ros2 topic list

# 监控特定话题
ros2 topic echo /joint_states

# 查看节点信息
ros2 node info /dummy_arm_controller_real

# 检查TF变换
ros2 run tf2_tools view_frames
```

## 8. 扩展开发

### 8.1 自定义控制器

继承基础控制器类，实现自定义功能：

```python
class CustomController(JointTrajectoryActionServer):
    def __init__(self):
        super().__init__()
        # 添加自定义功能
        
    def custom_move_function(self):
        # 实现自定义运动逻辑
        pass
```

### 8.2 添加新的演示程序

在`setup.py`中添加新的入口点：

```python
entry_points={
    'console_scripts': [
        'my_custom_demo = dummy_controller.my_custom_demo:main',
    ],
},
```

### 8.3 集成其他传感器

添加传感器数据处理：

```python
# 添加摄像头、激光雷达等传感器支持
self.sensor_sub = self.create_subscription(
    SensorData, '/sensor_topic', self.sensor_callback, 10)
```

## 9. 总结

本系统提供了完整的机械臂控制解决方案，包括：

- **硬件接口：** 直接控制物理机械臂
- **运动规划：** MoveIt2智能路径规划
- **可视化：** RViz实时显示和交互
- **编程接口：** Python和C++开发接口
- **安全保障：** 完整的安全检查机制
- **扩展性：** 易于添加新功能和传感器

系统设计模块化，易于理解和扩展，适合教学、研究和工业应用。 