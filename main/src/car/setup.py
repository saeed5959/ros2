from setuptools import find_packages, setup

package_name = 'car'

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
    maintainer='saeed',
    maintainer_email='saeedmr881@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_service = car.camera_node:main',
            'lidar_service = car.lidar_node:main',
            'perception_service = car.perception_node:main',
            'head_client = car.head:main'
        ],
    },
)
