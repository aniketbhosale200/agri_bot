import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    smdw_robot_base_dir = get_package_share_directory("smdw_robot_base")
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(smdw_robot_base_dir, "urdf", "smdw_robot_base.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=os.path.join(smdw_robot_base_dir, "rviz", "right_smdw_config"),
        description="Absolute path to RViz configuration file"
    )
    
    rviz_arg = DeclareLaunchArgument(
        name="rviz",
        default_value="true",
        description="Start RViz"
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=os.path.join(smdw_robot_base_dir, "worlds", "my_world.world"),
        description="Path to world file"
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation time"
    )
    
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        description="Start Joint State Publisher GUI"
    )
    
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    joint_state_publisher_node = Node(
        condition=UnlessCondition(LaunchConfiguration("gui")),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen"
    )
    
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(LaunchConfiguration("gui")),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen"
    )
    
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.path.join(smdw_robot_base_dir, "models")
    )
    
    gazebo_launch_path = os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            "verbose": "true",
            "world": LaunchConfiguration("world"),
            "extra_gazebo_args": "--max_step_size 0.001 --real_time_update_rate 1000 --physics_engine ode"
        }.items()
    )
    
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "/robot_description", "-entity", "smdw_robot_base", "-x", "0", "-y", "0", "-z", "0.5"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    spawn_entity = TimerAction(
        period=10.0,  # Increased delay
        actions=[spawn_entity_node]
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    
    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        rviz_arg,
        world_arg,
        use_sim_time_arg,
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        gazebo_model_path,
        gazebo,
        spawn_entity,
        rviz_node
    ])
