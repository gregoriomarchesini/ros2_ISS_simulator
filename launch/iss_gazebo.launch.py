import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


import xacro

def generate_launch_description():
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'iss_description'

    robot_xacro_subpath   = 'description/urdf/iss_model.xacro'
    world_file_subpath    = 'worlds/mocap_world.world'
    config_folder_subpath = 'config'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), robot_xacro_subpath)
    robot_description_config = xacro.process_file(xacro_file,mappings={"namespace":"iss"})
    robot_desc = robot_description_config.toxml()  
    
    # Reading initial conditions from YAML file
   
    world_path = os.path.join(get_package_share_directory(pkg_name),world_file_subpath)
    
    world = LaunchConfiguration('world') # this is a configuration (an input argument) to the launch file
    
    # This is to create a launch file input that can be used from command line to set the world file 
    # example (ros2 laucnh sml_nexus_description sml_nexus_gazebo.launch.py world:=path/to/your/world)
    # note that the variable world=  LaunchConfiguration('world') will get this value one given as input by the user
    declare_world_cmd = DeclareLaunchArgument(
	    name='world',
	    default_value=world_path,
	    description='Full path to the world model file to load')
	    
	    
    # Add a launch argument for verbose option
    verbose_arg = DeclareLaunchArgument('verbose', 
                                        default_value='false', 
                                        description='Set to true to enable verbose output')
    

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
)

    # Specify the actions

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments=[('verbose', 'true'),('world', world)],  # Pass verbose argument to included launch file
    )
   
    args =  ['-topic', '/robot_description', 
             '-entity', 'iss', 
             '-z', '2.0']
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=args,  # Separate arguments as individual strings
        output='screen',
        condition=IfCondition(LaunchConfiguration('verbose'))
    )
    


    return LaunchDescription([
        verbose_arg,  # Add the verbose argument
        declare_world_cmd,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
    ])
