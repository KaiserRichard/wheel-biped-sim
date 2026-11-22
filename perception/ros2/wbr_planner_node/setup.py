from setuptools import find_packages, setup

package_name = 'wbr_planner_node'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planner.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='WBR Research',
    maintainer_email='',
    description='Advice-only local lateral path suggestion from RGB-D and ORB-SLAM3 pose.',
    license='MIT',
    entry_points={'console_scripts': ['planner_node = wbr_planner_node.node:main']},
)
