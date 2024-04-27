from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'iss_description'


# do not remove these links
data_files=[
 ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
 ('share/' + package_name, ['package.xml'])]

# add here extra files and directories
data_files += [
        ('share/' + package_name + '/description/urdf', glob('description/urdf/*')),
        ('share/' + package_name + '/description/meshes', glob('description/meshes/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config/rviz2', glob('config/rviz2/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
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
