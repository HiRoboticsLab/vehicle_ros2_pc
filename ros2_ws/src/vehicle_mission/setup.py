from setuptools import setup
# ++ begin ++
from glob import glob
# ++ end ++

package_name = 'vehicle_mission'
# ++ begin ++
topic = 'vehicle_mission/topic'
# ++ end ++

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, topic],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ++ begin ++
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.lua')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz'))
        # ++ end ++
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guyulong',
    maintainer_email='guyulong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gogogo = vehicle_mission.gogogo:main',
        ],
    },
)
