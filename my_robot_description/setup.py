from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Install Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Install URDF Files (Include .xacro)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        
        # 3. Install Config Files (Include .yaml)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # 4. Install RViz Files (Optional, if you have the folder)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranav',
    maintainer_email='pranav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 5. Create the "ros2 run" executable
            'swerve_controller = my_robot_description.swerve_controller:main',
        ],
    },
)
