import os
from glob import glob
from setuptools import setup

package_name = 'px4_sitl_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools', 'numpy', 'control'],
    zip_safe=True,
    maintainer='Braden',
    maintainer_email='braden@arkelectron.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = px4_offboard.offboard_control:main',
                'visualizer = px4_offboard.visualizer:main',
                'velocity_control = px4_offboard.velocity_control:main',
                'control = px4_offboard.control:main',
                'processes = px4_offboard.processes:main',
                'get_servo_values = px4_offboard.get_servo_values:main',
                'set_servo_values = px4_offboard.set_servo_values:main',
                'set_q_values = px4_offboard.set_q_values:main',
                'set_vel_values = px4_offboard.set_vel_values:main',
                'hover_example = px4_offboard.hover_example:main',
                'fly_to = px4_offboard.fly_to:main',
                'px4_method = px4_offboard.px4_method:main',
                'px4_method_v2 = px4_offboard.px4_method_v2:main',
                'px4_method_v3 = px4_offboard.px4_method_v3:main',
                'serial_coms = px4_offboard.serial_coms:main',
        ],
    },
)
