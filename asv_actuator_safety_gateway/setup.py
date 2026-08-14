from setuptools import find_packages, setup

package_name = 'asv_actuator_safety_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mavros_hil_plugins.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Assignment submission',
    maintainer_email='maintainer@example.com',
    description='VRX actuator safety gateway built on MAVROS.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'safety_gateway = asv_actuator_safety_gateway.safety_gateway_node:main',
            'mavlink_hil_producer = asv_actuator_safety_gateway.mavlink_hil_producer:main',
            'stock_direct_producer = asv_actuator_safety_gateway.stock_direct_producer:main',
        ],
    },
)
