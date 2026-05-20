from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'haply_daVinci_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
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
            'haply_daVinci_bridge_node_jointcontrol = haply_daVinci_teleop.bridge.haply_daVinci_bridge_node_jointcontrol:main',    
            'haply_daVinci_bridge_node_jointcontrol_demo = haply_daVinci_teleop.bridge.haply_daVinci_bridge_node_jointcontrol_demo:main',
            'haptic_loop = haply_daVinci_teleop.haptic_visualization.haptic_loop:main',
            'haptic_hotwire = haply_daVinci_teleop.haptic_visualization.haptic_hotwire:main',
            'haptic_hotwire_demo = haply_daVinci_teleop.haptic_visualization.haptic_hotwire_demo:main',
            'study_controller = haply_daVinci_teleop.data_evaluation.study_controller:main',
            'study_controller_demo = haply_daVinci_teleop.data_evaluation.study_controller_demo:main',
        ],
    },
)
