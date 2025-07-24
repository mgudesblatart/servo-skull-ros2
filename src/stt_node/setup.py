from glob import glob
import os
from setuptools import find_packages, setup

package_name = "stt_node"

model_files = [
    f for f in glob(os.path.join("models", "sherpa-onnx-streaming-zipformer-en-2023-06-21/*"), recursive=True) if os.path.isfile(f)
]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            f"share/{package_name}/models",
            model_files,
        ),
    ],
    install_requires=["setuptools", "sherpa-onnx", "numpy", "rclpy"],
    zip_safe=True,
    maintainer="murray",
    maintainer_email="mgudesblatart@gmail.com",
    description="ROS2 node for streaming speech-to-text using Sherpa-ONNX Zipformer.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stt_node = stt_node.stt_node:main",
        ],
    },
)
