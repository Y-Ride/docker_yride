from setuptools import find_packages, setup

package_name = 'lla_ref'

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
    maintainer='Yride Golf Cart',
    maintainer_email='yridegolfcart@gmail.com',
    description='Packages used to average lla info to get a set reference',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lla_ref_node = lla_ref.lla_ref_node:main'
        ],
    },
)
