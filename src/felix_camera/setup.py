from glob import glob

from setuptools import find_packages, setup

package_name = 'felix_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Williams',
    maintainer_email='kevin.williams@sidusa.com',
    description='CSI IMX219 camera driver publishing CompressedImage.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = felix_camera.camera_node:main',
        ],
    },
)
