import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'assistant'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='Matthew Cromer',
    maintainer_email='matthew@example.com',
    description='ROS2 voice-controlled home assistant',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_node = assistant.speech_node:main',
            'trigger_node = assistant.trigger_node:main',
            'llm_node = assistant.llm_node:main',
        ],
    },
)
