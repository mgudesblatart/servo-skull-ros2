from setuptools import find_packages, setup

package_name = 'skull_control_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/testing.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'langchain',
        'llama_ros',
        'langchain_core',
        'langgraph',
        'ament_index_python',
    ],
    zip_safe=True,
    maintainer='murray',
    maintainer_email='mgudesblatart@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'skull_control_node = skull_control_node.skull_control_bt_node:main',
        ],
    },
)
