import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    world_path = '/home/ubuntu/ros2_ws/src/so100_simulation/worlds/table_obstacle.sdf'
    urdf_path = '/home/ubuntu/ros2_ws/src/SO-ARM100/Simulation/SO100/so100.urdf'
    set_env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', '/home/ubuntu/ros2_ws/src/SO-ARM100/Simulation/SO100')
    gz_sim = ExecuteProcess(cmd=['ign', 'gazebo', '-r', world_path], output='screen')
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-file', urdf_path, '-name', 'so100'], output='screen'
    )
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
    ]
    for j in ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper_joint']:
        bridge_args.append(f'/cmd_pos/{j}@std_msgs/msg/Float64]ignition.msgs.Double')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=bridge_args, output='screen', parameters=[{'use_sim_time': True}]
    )
    return LaunchDescription([set_env, gz_sim, spawn_robot, bridge])
