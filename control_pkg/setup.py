from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonas',
    maintainer_email='jmsh23@student.aau.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = control_pkg.location_test_publisher:main',
            'ref_talker = control_pkg.ref_test_publisher:main',
            'listener = control_pkg.control_signal_test_subscriber:main',
            'controller = control_pkg.controller:main',
            'updater = control_pkg.location_updater',
        ],
    },
)
