from setuptools import setup

package_name = 'mecanum_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='janikbalint',
    maintainer_email='janikbalintaron@gmail.com',
    description='Teleoperation node for mecanumbot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_teleop_key = mecanum_teleop.mecanum_teleop_key:main'
        ],
    },
)
