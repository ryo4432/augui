import os
from glob import glob
from setuptools import setup

package_name = 'augui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', 'assets'), glob('assets/*.html')),
        (os.path.join('share', 'assets'), glob('assets/css/*.css')),
        (os.path.join('share', 'assets'), glob('assets/css/*.css.map')),
        (os.path.join('share', 'assets'), glob('assets/js/*.js')),
        (os.path.join('share', 'assets'), glob('assets/js/*.js.map')),
        (os.path.join('share', 'assets'), glob('assets/js/charts/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryo4432',
    maintainer_email='ryoyoshimitsu@gmail.com',
    description='GUI for AUV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'augui = augui.augui:main',
            'mockup = augui.mockup:main'
        ],
    },
)
