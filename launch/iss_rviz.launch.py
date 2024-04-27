import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'iss_description'
    file_subpath = 'description/urdf/iss_model.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath) # looks into /install
    robot_description_raw = xacro.process_file(xacro_file).toxml()



    # Parameter node: stores all the parameters from the urdf file that can be read by rviz for example. )
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )
    
    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory(pkg_name), 'config/rviz2', 'rviz_config.rviz')]
            )

    
    # this node simplu publishes a joint_state message. The message send the a transfor message (tf2) such that the 
    # robot model can be constructued form the base link to the rest of the links. Without this information 
    # the model cannot be directly reconstructed. 
    joints_state_publisher = Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher",
                )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        joints_state_publisher,
        rviz
    ])
