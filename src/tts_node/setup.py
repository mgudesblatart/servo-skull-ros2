from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'tts_node'

model_files = [
    f for ext in ("*.onnx", "*.json") for f in glob(os.path.join("models", ext)) if os.path.isfile(f)
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
          (
            f"share/{package_name}/models",
            model_files,
        ),
    ],
    install_requires=['setuptools', "rclpy", "std_msgs", "servo_skull_msgs", "piper-tts", "numpy"],
    zip_safe=True,
    maintainer='murray',
    maintainer_email='mgudesblatart@gmail.com',
    description='ROS2 node for text-to-speech using Piper TTS.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tts_node = tts_node.tts_node:main",
        ],
    },
)
