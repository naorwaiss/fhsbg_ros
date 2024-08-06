from setuptools import setup

package_name = 'fhsbg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_node.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'dist_with_ros = fhsbg.dist_with_ros:main',
            'camera_modification = fhsbg.camera_modification:main',
            'aruco_filter = fhsbg.aruco_filter:main',
            'main_ros = fhsbg.main_ros:main',

        ],
    },
)
