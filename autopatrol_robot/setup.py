from setuptools import find_packages, setup
# glob是用来查找符合特定规则的文件路径名的模块
from glob import glob

package_name = 'autopatrol_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 路径由share/autopatrol_robot/launch 和 share/autopatrol_robot/config 组成
        ('share/' + package_name+"/launch", glob('launch/*.launch.py')),
        ('share/' + package_name+"/config", ['config/patrol_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cheems',
    maintainer_email='yuweihe5618@163.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node = autopatrol_robot.patrol_node:main',
            'speaker = autopatrol_robot.speaker:main',
        ],
    },
)
