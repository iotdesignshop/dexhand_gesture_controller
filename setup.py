from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'dexhand_gesture_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        
  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trent Shumay',
    maintainer_email='trent@iotdesignshop.com',
    description='A high-level semantic controller for the DexHand',
    license='CC BY-NC-SA 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_controller = dexhand_gesture_controller.gesture_controller:main'
        ],
    },
)
