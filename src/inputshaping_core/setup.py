from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = 'inputshaping_core'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE_NAME + '/templates', glob('templates/*.html')),
        ('share/' + PACKAGE_NAME + '/static', glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mikhail Skvortcov',
    maintainer_email='mikhail.skvortcov@example.com',
    description='Input shaping bench for a mobile robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'experiment_node = inputshaping_core.experiment_node:main',
            'mpu_serial_node = inputshaping_core.mpu_serial_node:main',
        ],
    },
)
