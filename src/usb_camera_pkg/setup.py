from setuptools import find_packages, setup

package_name = 'usb_camera_pkg'

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
    maintainer='weng',
    maintainer_email='wengkunduo@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_cam_rgb_node = usb_camera_pkg.usb_camera_rgb_node:main',
        ],
    },
)
