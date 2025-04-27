from setuptools import find_packages, setup
from glob import glob
package_name = 'sim_dynamics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
        glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='prajwalthakur98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'sim_dynamics_node = sim_dynamics.sim_dynamics_node:main'
            'sim_robot_node = sim_dynamics.sim_robot_node:main',
            'update_dynamics_node = sim_dynamics.update_dynamics_node:main',
        ],
    },
)
