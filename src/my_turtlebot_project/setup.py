from setuptools import find_packages, setup

package_name = 'my_turtlebot_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ### Add launch directory and files ###
        ('share/' + package_name + '/launch', ['launch/turtlebot_simulation.launch.py']),
        ### Add maps fils ###
        ('share/' + package_name + '/maps', ['maps/dummy_map.yaml', 'maps/dummy_map.pgm']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yjh',
    maintainer_email='y6hyuk@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'mapping_gui = my_turtlebot_project.mapping_gui:main',
            'spawn_controller = my_turtlebot_project.spawn_controller_node:main'
        ],
    },
)
