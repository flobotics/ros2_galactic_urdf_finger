from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'ros2_galactic_urdf_finger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('meshes/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='inflo@web.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = ros2_galactic_urdf_finger.state_publisher:main'
        ],
        'console_scripts': [
            'angle_finger_test_state_publisher = ros2_galactic_urdf_finger.angle_finger_test_state_publisher:main'
        ],
        'console_scripts': [
            'hand_21_points_state_publisher = ros2_galactic_urdf_finger.hand_21_points_state_publisher:main'
        ],
    },
)
