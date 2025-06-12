from setuptools import setup, find_packages

package_name = 'robot_arm_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_arm_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_arm',
    maintainer_email='YOUR_EMAIL@example.com',
    description='2-DOF Robot Arm Controller for MCP integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = robot_arm_controller.motor_controller:main',
            'joint_state_publisher = robot_arm_controller.joint_state_publisher:main',
            'cartesian_controller = robot_arm_controller.cartesian_controller:main',
        ],
    },
) 