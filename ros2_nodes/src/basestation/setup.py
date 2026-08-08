import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'basestation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'basestation': [
            'templates/*.html',
            'static/*.svg',
            'static/*.css',
            'static/icons/*.svg',
            'static/logo/*.png',
            'images/*.jpg',
            'assets/video/*.mp4',
        ],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humberasv',
    maintainer_email='mechatronicsclub@humber.ca',
    description='Loon-E basestation operation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'flask = basestation.app:main',
            'stream = basestation.annotated_udp_stream:main'
        ],
    },
)
