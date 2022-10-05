from setuptools import setup

package_name = 'robot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds/', ['worlds/my_world.wbt']),
        ('share/' + package_name + '/resource/', ['resource/my_robot.urdf']),
        ('share/' + package_name + '/launch/', ['launch/robot_launch.py']),
        ('share/' + package_name + '/config/', ['config/my_robot_lds_2d.lua', 'config/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='summo',
    maintainer_email='aprlagare1999@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = robot_simulation.my_robot_driver:main',
            'odom_estimator = robot_simulation.odom_estimator:main',
        ],
    },
)
