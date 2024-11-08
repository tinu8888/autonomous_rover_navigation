from setuptools import find_packages, setup

package_name = 'rover_motion'

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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'direct_coordinate = rover_motion.direct_coordinate:main',
            'accurate_odo = rover_motion.accurate_odo:main',
            'check = rover_motion.check:main',
            'motion_node = rover_motion.motion_node:main',
            'newcontrol = rover_motion.newcontrol:main',
            'speech_movement = rover_motion.speech_movement:main',
            'swim_to_goal = rover_motion.swim_to_goal:main',
            'control_node = rover_motion.control_node:main',
            'serial_com = rover_motion.serial_com:main',
            'odom_node = rover_motion.odom_node:main',
            'gps_loc = rover_motion.gps_loc:main',
            'coordinate_gps = rover_motion.coordinate_gps:main',
        ],
    },
)
