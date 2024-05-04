import os
from rclpy.logging import get_logger
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


import subprocess
import xacro




# if you are not familiar with launch files please visit 
# https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/

# also visit this repo to understand why OpaqueFunctions are used in this case
# https://github.com/jrgnicho/collaborative-robotic-sanding/blob/3902e4f0e76bde226b18a997fd60fc30e1961212/crs_application/launch/perception.launch.py#L21
# the main reason : there is not way to access the value of the LaunchArguments as a string unless you have a context object. Hopefully this will bchanged in the future developments of ros2



def setup_launch(context,*args, **kwargs) :
    # you need an opaque fuction to access the value of the arguments as a string. It is a bit convoluted but this is the only way
    
    logger = get_logger("spawn_target_launch")
    model_name = LaunchConfiguration('model').perform(context=context)
    world_name = LaunchConfiguration('world').perform(context=context)
    
    
    pkg_name = 'simulator_ros2'
    robot_xacro_subpath = 'description/'+model_name+'/urdf/model/model.xacro'
    world_file_subpath = 'worlds/'+world_name+'.world'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), robot_xacro_subpath)
    robot_description_config = xacro.process_file(xacro_file, mappings={"namespace": model_name})
    robot_desc = robot_description_config.toxml()
    world_path = os.path.join(get_package_share_directory(pkg_name), world_file_subpath)

    # Execute pgrep command to check if gzserver is running
    try:
        result = subprocess.run(['pgrep', '-x', 'gzserver'], stdout=subprocess.PIPE, check=True) # will give an error if gazebo is not running
        gazebo_running = True
    
    except subprocess.CalledProcessError: # this is the error given by subprocess.run
        gazebo_running = False
    

    if gazebo_running:
        logger.info("Gazebo server is running already... selected wold file will not be used.")
    
    # gazebo is spawned only if it is not already running
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments=[('world', world_path,'real_time_factor',10.0)])

    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=model_name,
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )
    
    args = ['-topic', '/'+model_name + '/robot_description', '-entity', model_name, '-z', '0.0','-x', '0.0','-y', '0.0'] # if you did not name the robot correctly in the model.xacro file, then nothing will be shown
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name = model_name + '_spawner', # this will affect the name space of all the nodes spawned within the plugins of the model
        arguments = args,
        output='screen',
    )
    
    if not gazebo_running:
        return [gazebo,node_robot_state_publisher, spawn_entity]
    else:
        return [node_robot_state_publisher, spawn_entity ]
        


def generate_launch_description():
    
    ld = []
   
    # configurations of the launch file arguments
    model_name_arg = DeclareLaunchArgument(name='model',default_value='ISS',description='Name of the model to spawn contained in the description folder (without the .xacro in the end, just the ma,e of the foler is sufficient). Default to the ISS') # declaration of the launch argument
    ld += [model_name_arg]
    
    world_name_arg = DeclareLaunchArgument( name='world', default_value='mocap_world', description='name of the world file to be spawned (without the .world in the end). If gazebo is already running, the prespawned world will be used intead' )
    ld += [world_name_arg]
    
    
    
    ld +=[OpaqueFunction(function = setup_launch)]

    return LaunchDescription(ld)

