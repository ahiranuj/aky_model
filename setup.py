from glob import glob
import os

from setuptools import setup


package_name = 'aky_model'


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anuj Kumar Yadav',
    maintainer_email='anujyadavji2007@gmail.com',
    description='AKY custom four-wheel UGV Xacro model and RViz teleoperation demo.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aky_motion = aky_model.ugv_motion:main',
        ],
    },
)
