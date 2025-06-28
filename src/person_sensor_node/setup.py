from setuptools import find_packages, setup

package_name = 'person_sensor_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add message files
        ('share/' + package_name + '/msg', ['msg/PersonDetection.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='murray',
    maintainer_email='murray@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_sensor_node = person_sensor_node.person_sensor_node:main',
        ],
    },
)
