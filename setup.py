from setuptools import setup

package_name = 'f1tenth_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/slam_toolbox.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/map.launch.py',
            'launch/process_raceline.launch.py',
            'launch/test.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bala Kolanu',
    maintainer_email='bkolanu@purdue.edu',
    description='F1TENTH mapping and autonomy orchestration for the current ROS 2 stack.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mapping_session_manager = f1tenth_bringup.mapping_session_manager:main',
        ],
    },
)
