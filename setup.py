from setuptools import setup

package_name = 'random_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/',
         ['launch/gazebo_launch.py', 'launch/simulator_launch.py', 'launch/turtlebot_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mridul',
    maintainer_email='noitsnotmridul@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_nav_node = random_nav.random_nav_node:main'
        ],
    },
)
