from setuptools import find_packages, setup

package_name = 'gazebo_ai'

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
    maintainer='jeremy',
    maintainer_email='jeremymcote@gmail.com',
    description='A Python client for interacting with Gazebo World Control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_control = gazebo_ai.world_control:main'
        ],
    },
)