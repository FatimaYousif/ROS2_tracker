import os
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnShutdown
from launch_ros.actions import Node

def kill_gazebo_processes(context):
    """Function to kill Gazebo and related processes"""
    print("Shutting down Gazebo and related processes...")
    subprocess.run(['pkill','-9' ,'-f', 'gz sim'])
    subprocess.run(['pkill', '-9', '-f', 'gz'])

    subprocess.run(['pkill','-9', '-f', 'px4'])
    subprocess.run(['pkill', '-f', 'QGroundControl'])
    subprocess.run(['pkill', '-9','-f', 'MicroXRCEAgent'])

def generate_launch_description():
    ld = LaunchDescription()

    ros2_ws_path = os.path.expanduser('~/ros2_ws')
    
    # 1. Micro XRCE-DDS Agent
    microxrce_agent = ExecuteProcess(
        cmd=['bash', '-c', 'cd ~/Micro-XRCE-DDS-Agent && MicroXRCEAgent udp4 -p 8888'],
        output='screen'
    )
    ld.add_action(microxrce_agent)

    # 2. PX4 SITL with Gazebo (after 3 seconds)
    px4_sitl = ExecuteProcess(
        cmd=['bash', '-c', 'cd ~/PX4-Autopilot && make px4_sitl gz_x500_depth'],
        output='screen'
    )
    ld.add_action(TimerAction(period=3.0, actions=[px4_sitl]))

    # 3. QGroundControl 
    qgroundcontrol = ExecuteProcess(
        cmd=['bash', '-c', 'cd ~/Downloads && ./QGroundControl-x86_64.AppImage'],
        # cmd=['bash', '-c', 'cd ~/Desktop && ./QGroundControl.AppImage'],
        output='screen'
    )
    ld.add_action(TimerAction(period=8.0, actions=[qgroundcontrol]))

    # 4. ROS-Gazebo bridges
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        arguments=['/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )

    camera_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_image_bridge',
        arguments=['/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )
    ld.add_action(TimerAction(period=20.0, actions=[camera_info_bridge]))
    ld.add_action(TimerAction(period=13.0, actions=[camera_image_bridge]))

    # 5. Ultralytics ROS tracker (after 18 seconds) + ROS2 workspace sourced
    # tracker_launch = ExecuteProcess(
    #     cmd=['bash', '-c', f'cd {ros2_ws_path} && source install/setup.bash && ros2 launch ultralytics_ros tracker.launch.xml debug:=false'],
    #     output='screen'
    # )
    # ld.add_action(TimerAction(period=18.0, actions=[tracker_launch]))

    # 6. Simple Python script (after 26 seconds)
    # simple_script = ExecuteProcess(
    #     cmd=['bash', '-c', f'cd {ros2_ws_path} && source install/setup.bash && cd src/target_tracking/target_tracking && python3 simple.py'],
    #     output='screen'
    # )
    # ld.add_action(TimerAction(period=26.0, actions=[simple_script]))

    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda context: kill_gazebo_processes(context))]
        )
    )
    ld.add_action(shutdown_handler)

    return ld
