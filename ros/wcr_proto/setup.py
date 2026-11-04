from glob import glob

from setuptools import setup

package_name = 'wcr_proto'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/resource', glob('resource/*.urdf') + glob('resource/*.json')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl') + glob('meshes/*.json')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['coverage_mission = wcr_proto.coverage_mission:main']},
    zip_safe=True,
    maintainer='Deepak-EL',
    maintainer_email='193407579+Deepak-EL@users.noreply.github.com',
    description='Digital twin of the physical WCR prototype (thrust adhesion, N20 skid steer) in Webots + ROS 2',
    license='Apache-2.0',
)
