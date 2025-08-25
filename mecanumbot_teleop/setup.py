from setuptools import find_packages
from setuptools import setup

package_name = 'mecanumbot_teleop'

setup(
    name=package_name,
    version='2.3.3',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='Fortuz',
    author_email='fortuz19@gmail.com',
    maintainer='Fortuz',
    maintainer_email='fortuz19@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Teleoperation node using keyboard for Mecanumbot.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'mecanumbot_keyboard = mecanumbot_teleop.script.teleop_keyboard:main'
        ],
    },
)
