from setuptools import setup
import os
from glob import glob

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- BU SATIRLARI EKLE ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # -------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senin_adin',
    maintainer_email='mail@mail.com',
    description='Rover baslatma paketi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Eski node varsa kalsın, altına bunu ekle:
            'empty_map_pub = rover_bringup.empty_map:main',
            'set_datum_auto = rover_bringup.set_datum_auto:main',
        ],
    },
)