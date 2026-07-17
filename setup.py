from setuptools import find_packages, setup

package_name = 'vessel_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_driver = vessel_system.test_driver:main',
            'pose_to_tf = vessel_system.pose_to_tf:main',
            'check_sonar_sensor = vessel_system.check_sonar_sensor:main',    
            'teleop_wasd = vessel_system.teleop_wasd:main',
            'gps_velocity = vessel_system.gps_velocity:main', 
            'controller_node = vessel_system.controller_node:main',    
            'motor_mixer = vessel_system.motor_mixer:main',
            'waypoint_manager_node = vessel_system.waypoint_manager_node:main',
        ],
    },
)
