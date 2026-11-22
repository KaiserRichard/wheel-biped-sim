from setuptools import find_packages, setup

package_name = 'wbr_obstacle_node'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/obstacle.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='WBR Research',
    maintainer_email='',
    description='Informational depth-based ground and obstacle advisory.',
    license='MIT',
    entry_points={'console_scripts': ['obstacle_node = wbr_obstacle_node.node:main']},
)
