from setuptools import find_packages, setup

package_name = 'speaker_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'sounddevice', 'scipy'],
    zip_safe=True,
    maintainer='murray',
    maintainer_email='mgudesblatart@gmail.com',
    description='ROS2 node for audio playback from AudioData messages.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              'speaker_node = speaker_node.speaker_node:main',
        ],
    },
)
