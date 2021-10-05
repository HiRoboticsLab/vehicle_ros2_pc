from setuptools import setup
# ++ begin ++
from glob import glob
# ++ end ++

package_name = 'vehicle_adjust'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ++ begin ++
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
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
            'pid = vehicle_adjust.pid:main',
            # 'time = vehicle_adjust.time:main'
        ],
    },
)
