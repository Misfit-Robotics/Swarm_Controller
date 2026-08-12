from setuptools import find_packages, setup

package_name = 'swarm_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/agent.launch.py', 'launch/swarm.launch.py']),
    ],
    package_data={'': ['py.typed'], 'swarm_control_pkg': ['templates/*']},
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='Sean Williams',
    maintainer_email='Sean.Williams.3@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'swarm_controller = swarm_control_pkg.swarm_controller:main',
            'swarm_cli = swarm_control_pkg.swarm_cli:main',
            'swarm_api = swarm_control_pkg.swarm_api:main',
        ],
    },
)
