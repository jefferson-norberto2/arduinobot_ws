from launch import LaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the pkg directory path
    arduino_bot_description_dir = get_package_share_directory("arduinobot_description")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(arduino_bot_description_dir, 
                                    "urdf", 
                                    "arduinobot.urdf.xacro"),
        description="Absolute path to the robot URDF file"
        )
    
    # Setting args to gazebo
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(arduino_bot_description_dir).parent.resolve())
        ]
    )

    # Transform declared launch on args
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    # Node to publish robot informations
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    # Start gazebo using python descritpion
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch"
            ), "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", ["-v 4 -r empty.sdf"])
        ]
    )


    # Spaw robot on gazebo world
    gz_spaw_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "arduinobot"]
    )

    # Call the bridge to use gazebo and ros2
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ]
    )
    
    # Put the rigth sequence to launch robot
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spaw_entity,
        gz_ros2_bridge
    ])