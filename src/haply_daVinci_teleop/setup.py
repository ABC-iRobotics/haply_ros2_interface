from setuptools import find_packages, setup

package_name = 'haply_daVinci_teleop'

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
    maintainer='student',
    maintainer_email='kiss.gergely1213@edu.bme.hu',
    description='This package was created to conduct a haptic force feedback teleoperation with the Haply device and the daVinci robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'haply_bridge_jointcontrol = haply_daVinci_teleop.bridge.haply_daVinci_bridge_node_jointcontrol:main',
            'haptic_ball_visualization = haply_daVinci_teleop.visualization.haptic_ball:main',      
            'haptic_loop = haply_daVinci_teleop.visualization.haptic_loop:main',
        ],
    },
)
