import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gimbal_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'weights'), ['weights/best.pt']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # SAFE URDF MATCHING (Klasörleri es geçer, sadece urdf ve xacro dosyalarını kopyalar)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali Talha Oruç',
    maintainer_email='alitalhaoruc@gmail.com',
    description='Gimbal Turret Description Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_mover = gimbal_description.target_mover:main',
            'target_detector = gimbal_description.target_detector:main',
            'target_tracker = gimbal_description.target_tracker:main',
        ],
    },
)
