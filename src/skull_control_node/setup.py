from setuptools import find_packages, setup
import glob

package_name = 'skull_control_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # Include all submodules
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/configs', glob.glob('configs/*.yaml') + glob.glob('configs/*.gbnf')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'langchain',
        'langchain_core',
        'langgraph',
        'ament_index_python',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='murray',
    maintainer_email='mgudesblatart@gmail.com',
    description='AXCL-backed LLM agent and BT-based servo skull controller for ROS2 Jazzy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'skull_control_node = skull_control_node.skull_control_bt_node:main',
            'llm_agent_axcl_node = skull_control_node.llm_agent_axcl_node:main',
        ],
    },
)
