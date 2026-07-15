from setuptools import find_packages, setup

package_name = 'rov_pixhawk_control'

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
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rov_pixhawk_control_node = rov_pixhawk_control.rov_pixhawk_control:main',
            'attitude_publisher = rov_pixhawk_control.attitude_publisher:main',
        ],
    },
)
