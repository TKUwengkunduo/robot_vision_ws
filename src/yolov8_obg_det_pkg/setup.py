from setuptools import find_packages, setup

package_name = 'yolov8_obg_det_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/model', ['model/yolov8n.pt']),  # 添加模型文件
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
            'yolov8_obg_det = yolov8_obg_det_pkg.yolov8_obg_det:main',  # 確保添加入口點
        ],
    },
)
