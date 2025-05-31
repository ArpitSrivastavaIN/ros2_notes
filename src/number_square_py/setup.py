import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'number_square_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arpit',
    maintainer_email='work.arpit.sri@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'number_publisher = number_square_py.number_publisher:main',
            'square_subscriber = number_square_py.square_subscriber:main',
        ],
    },
)
