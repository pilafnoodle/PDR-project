# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():

#     port_argument = DeclareLaunchArgument(
#         'port',
#         default_value='/dev/ttyUSB0',        
#         description='port that imu is on'
#     )
#     imu_node=Node(
#         package='data_collector',
#         executable='imu_driver',
#         name='imu_driver',
#         parameters=[{'port':LaunchConfiguration('port')}]
#     )

#     gps_node=Node(
#         package='data_collector',
#         executable='gps_driver',
#         name='gps_driver',
#         parameters=[{'port':LaunchConfiguration('port')}]
#     )
#     labeling_node=Node(
#         package='data_collector',
#         executable='labeling_node',
#         name='labeling_node',
#     )
#     return LaunchDescription([
#         port_argument,
#         imu_node,
#         labeling_node
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # gps_port_argument = DeclareLaunchArgument(
    #     'gps_port',
    #     default_value='/dev/ttyUSB0',        
    #     description='port that gps puck is on'
    # )

    # gps_node=Node(
    #     package='data_collector',
    #     executable='gps_driver',
    #     name='gps_driver',
    #     parameters=[{'port':LaunchConfiguration('gps_port')}]
    # )
    imu_port_argument = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyUSB0',        
        description='port that imu is on'
    )
    imu_node=Node(
        package='data_collector',
        executable='imu_driver',
        name='imu_driver',
        parameters=[{'port':LaunchConfiguration('imu_port')}]
    )


    return LaunchDescription([
        imu_port_argument,
        imu_node
    ])