from setuptools import setup

package_name = 'my_gazebo_spawner'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_gazebo_spawner = my_gazebo_spawner.gazebo_spawner:main',
            'my_gazebo_spawner_v0 = my_gazebo_spawner.gazebo_spawner_v0:main',
        ],
    },
)
