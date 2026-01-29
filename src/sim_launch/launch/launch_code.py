import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # -----------------------------
    # 1. Paths to URDF and world
    # -----------------------------
    pkg_path = get_package_share_directory('rov_sim')
    urdf_file = os.path.join(pkg_path, 'urdf', 'omni_rov.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'underwater.world')

    # -----------------------------
    # 2. Include Gazebo simulation
    # -----------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),  # -r ensures world reset
    )

    # -----------------------------
    # 3. Spawn ROV (with --replace)
    # -----------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'omni_rov',
            '-file', urdf_file,
            '-z', '2.0',
        ],
        output='screen',
    )

    # -----------------------------
    # 4. Bridge ROS 2 <-> Gazebo for all 8 thrusters
    # -----------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/rov/thruster/t0@std_msgs/msg/Float64@gz.msgs.Double',
            '/rov/thruster/t1@std_msgs/msg/Float64@gz.msgs.Double',
            '/rov/thruster/t2@std_msgs/msg/Float64@gz.msgs.Double',
            '/rov/thruster/t3@std_msgs/msg/Float64@gz.msgs.Double',
            '/rov/thruster/t4@std_msgs/msg/Float64@gz.msgs.Double',
            '/rov/thruster/t5@std_msgs/msg/Float64@gz.msgs.Double',
            '/rov/thruster/t6@std_msgs/msg/Float64@gz.msgs.Double',
            '/rov/thruster/t7@std_msgs/msg/Float64@gz.msgs.Double',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/pressure@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure'
        ],
        output='screen'
    )

    # -----------------------------
    # 5. Launch everything
    # -----------------------------
    return LaunchDescription([
        gz_sim,
        spawn_robot,
        bridge
    ])
