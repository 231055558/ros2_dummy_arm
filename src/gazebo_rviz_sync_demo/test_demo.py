#!/usr/bin/env python3
"""
测试脚本 - 验证 gazebo_rviz_sync_demo 包是否正确安装
"""

import subprocess
import sys
import os
from ament_index_python.packages import get_package_share_directory

def test_package_installation():
    """测试包是否正确安装"""
    print("🔍 测试 gazebo_rviz_sync_demo 包安装...")
    
    try:
        # 测试包路径
        pkg_share = get_package_share_directory('gazebo_rviz_sync_demo')
        print(f"✅ 包路径: {pkg_share}")
        
        # 检查关键文件
        files_to_check = [
            'worlds/simple_cube.world',
            'launch/sync_demo.launch.py',
            'rviz/cube_sync.rviz'
        ]
        
        for file_path in files_to_check:
            full_path = os.path.join(pkg_share, file_path)
            if os.path.exists(full_path):
                print(f"✅ 文件存在: {file_path}")
            else:
                print(f"❌ 文件缺失: {file_path}")
                return False
        
        # 测试可执行文件
        executables = ['cube_synchronizer', 'cube_controller']
        for exe in executables:
            try:
                result = subprocess.run(
                    ['ros2', 'pkg', 'executables', 'gazebo_rviz_sync_demo'],
                    capture_output=True, text=True, timeout=10
                )
                if exe in result.stdout:
                    print(f"✅ 可执行文件: {exe}")
                else:
                    print(f"❌ 缺失可执行文件: {exe}")
                    return False
            except subprocess.TimeoutExpired:
                print(f"⚠️  超时检查可执行文件: {exe}")
        
        print("🎉 所有测试通过！包安装正确。")
        return True
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False

def print_usage_instructions():
    """打印使用说明"""
    print("\n📖 使用说明:")
    print("=" * 50)
    print("1. 启动完整演示:")
    print("   ros2 launch gazebo_rviz_sync_demo sync_demo.launch.py")
    print()
    print("2. 不同移动模式:")
    print("   ros2 launch gazebo_rviz_sync_demo sync_demo.launch.py movement_mode:=square")
    print("   ros2 launch gazebo_rviz_sync_demo sync_demo.launch.py movement_mode:=line")
    print()
    print("3. 仅同步，不自动移动:")
    print("   ros2 launch gazebo_rviz_sync_demo sync_demo.launch.py auto_move:=false")
    print()
    print("4. 手动控制方块:")
    print("   ros2 topic pub /demo_cube/cmd_vel geometry_msgs/msg/Twist \\")
    print("   \"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\"")
    print()
    print("5. 监控话题:")
    print("   ros2 topic echo /demo_cube/odom")
    print("   ros2 topic echo /cube_markers")
    print("=" * 50)

if __name__ == '__main__':
    print("🚀 Gazebo-RViz 同步演示包测试")
    print("=" * 50)
    
    if test_package_installation():
        print_usage_instructions()
        sys.exit(0)
    else:
        print("\n❌ 安装测试失败！请检查编译过程。")
        sys.exit(1) 