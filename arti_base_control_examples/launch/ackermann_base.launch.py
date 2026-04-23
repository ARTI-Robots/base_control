from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Declare launch arguments ---
    respawn = LaunchConfiguration('respawn')
    max_velocity_linear = LaunchConfiguration('max_velocity_linear')
    max_velocity_angular = LaunchConfiguration('max_velocity_angular')
    max_steering_angle = LaunchConfiguration('max_steering_angle')
    allowed_brake_velocity = LaunchConfiguration('allowed_brake_velocity')
    brake_velocity = LaunchConfiguration('brake_velocity')
    brake_current = LaunchConfiguration('brake_current')
    odometry_rate = LaunchConfiguration('odometry_rate')
    publish_motor_states = LaunchConfiguration('publish_motor_states')
    use_mockup = LaunchConfiguration('use_mockup')

    # --- Package paths ---
    pkg_share = FindPackageShare('arti_base_control_examples')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'ackermann_base.urdf.xacro'])
    config_file = PathJoinSubstitution([pkg_share, 'config', 'ackermann_base.yaml'])

    # --- Generate robot_description using xacro ---
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ', # add space to avoid concatenation of xacro and urdf_file which would cause an error
        urdf_file
    ])

    # --- robot_state_publisher node ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        respawn=True,
        parameters=[
            {'robot_description': robot_description_content},
            {'publish_frequency': 30.0}
        ]
    )

    # --- arti_base_control node ---
    base_control_node = Node(
        package='arti_base_control',
        executable='arti_base_control_node',  # change if needed
        name='base_control',
        output='screen',
        respawn=respawn,
        parameters=[
            config_file,  # YAML configuration file
            {
                'odometry_rate': odometry_rate,
                'publish_motor_states': publish_motor_states,
                'use_mockup': use_mockup,

                # Vehicle configuration
                'vehicle.max_velocity_linear': max_velocity_linear,
                'vehicle.max_velocity_angular': max_velocity_angular,
                'vehicle.max_steering_angle': max_steering_angle,
                'vehicle.allowed_brake_velocity': allowed_brake_velocity,
                'vehicle.brake_velocity': brake_velocity,
                'vehicle.brake_current': brake_current
            }
        ]
    )

    # --- Final launch description ---
    return LaunchDescription([
        DeclareLaunchArgument('respawn', default_value='false'),
        DeclareLaunchArgument('max_velocity_linear', default_value='2.0'),
        DeclareLaunchArgument('max_velocity_angular', default_value='1.5708'),
        DeclareLaunchArgument('max_steering_angle', default_value='0.5'),
        DeclareLaunchArgument('allowed_brake_velocity', default_value='0.001'),
        DeclareLaunchArgument('brake_velocity', default_value='0.001'),
        DeclareLaunchArgument('brake_current', default_value='1.0'),
        DeclareLaunchArgument('odometry_rate', default_value='10.0'),
        DeclareLaunchArgument('publish_motor_states', default_value='false'),
        DeclareLaunchArgument('use_mockup', default_value='true'),

        robot_state_publisher_node,
        base_control_node
    ])