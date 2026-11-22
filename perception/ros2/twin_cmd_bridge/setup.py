from setuptools import find_packages, setup

setup(
    name='twin_cmd_bridge', version='0.1.0',
    packages=find_packages(exclude=('tests',)),
    data_files=[('share/ament_index/resource_index/packages', ['resource/twin_cmd_bridge']),
                ('share/twin_cmd_bridge', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='WBR Research', maintainer_email='', license='MIT',
    description='AGV-style /cmd_vel rehearsal to a Mac physics twin via UDP.',
    entry_points={'console_scripts': ['bridge = twin_cmd_bridge.node:main']},
)
