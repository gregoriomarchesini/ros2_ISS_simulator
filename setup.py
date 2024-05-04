from setuptools import find_packages, setup
import os
from glob import glob
import logging

package_name = 'simulator_ros2'


# do not remove these links
data_files=[
 ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
 ('share/' + package_name, ['package.xml'])]



# Configure logging for a specific logger named 'setup_logger'
setup_logger = logging.getLogger('SETUP_LOGGER')
setup_logger.setLevel(logging.INFO)  # Set the logging level to INFO for this logger
# Add a console handler to output logs to stderr (console)
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)  # Set the handler level to INFO
setup_logger.addHandler(console_handler)  # Attach the handler to the logger

# Log information about found models using the named logger
models = os.listdir("description/")
setup_logger.info("\n")
for model in models:
    setup_logger.info("Found model: %s", model)
    
for model in models:
    data_files += [
        ('share/' + package_name + '/description/'+model+'/urdf/model'  , glob('description/'+model+'/urdf/model/*')),
        ('share/' + package_name + '/description/'+model+'/urdf/gazebo'  , glob('description/'+model+'/urdf/gazebo/*')),
        ('share/' + package_name + '/description/'+model+'/meshes'       , glob('description/'+model+'/meshes/*')),
    ]


# add here all the files and directories to be shared within your package and workspace (symlink located at install/share)
data_files += [
        ('share/' + package_name + '/launch'                   , glob('launch/*')),
        ('share/' + package_name + '/config/rviz2'             , glob('config/rviz2/*')),
        ('share/' + package_name + '/worlds'                   , glob('worlds/*')),
]


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your@email.com',
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Your package description',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'your_node_name = your_package_name.your_module_name:main'
        ],
    },
)
