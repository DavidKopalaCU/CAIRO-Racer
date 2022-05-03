from setuptools import setup

package_name = 'deepracer_webots'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    (f'share/{ package_name }/launch', ['launch/deepracer_webots.launch.py']),
    (f'share/{ package_name }/worlds', ['worlds/tutorial_world.wbt']),
    (f'share/{ package_name }/resource', ['resource/robot.urdf']),
    (f'share/{ package_name }/protos', ['protos/Agent.proto']),
    (f'share/{ package_name }', ['package.xml'])
]


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davidkopala',
    maintainer_email='kopala.david@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = deepracer_webots.my_robot_driver:main',
            'camera_shim = deepracer_webots.camera_shim:main'
        ],
    },
)
