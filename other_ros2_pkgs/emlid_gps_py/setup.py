from setuptools import find_packages, setup

package_name = 'emlid_gps_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ben Boyack',
    maintainer_email='yridegolfcart@gmail.com',
    description='Python version to read, parse, and publish nmea strings from serial port. Also publishes relative information given origin.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emlid_gps_py = emlid_gps_py.emlid_gps_py:main'
        ],
    },
)
