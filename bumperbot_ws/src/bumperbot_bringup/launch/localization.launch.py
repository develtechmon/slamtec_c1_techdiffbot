import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the launch directory
    package_name = 'bumperbot_bringup'    

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory(package_name), 'map', 'map_v1.yaml')

    #default_bt_xml_filename= os.path.join(get_package_share_directory('agv_proto'), 'config', 'navigate_w_replanning_and_recovery.xml')
    
    lifecycle_nodes = ['map_server', 
                       'amcl'
                       ]

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename':map_file}],
            remappings=remappings),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml],
            remappings=remappings,
            ),
                     
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]),
    ])