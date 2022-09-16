from setuptools import setup

package_name = 'my_point_cloud2'

setup(
    name=package_name,
    version='2.0.5',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='jamesnail333@gmail.com',
    description='A package for easy creation and reading of PointCloud2 messages in Python.',
    license='BSD',
    tests_require=['pytest'],
)
# https://github.com/ros2/common_interfaces/blob/foxy/sensor_msgs_py/setup.py#L20