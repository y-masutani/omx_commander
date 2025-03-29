from setuptools import find_packages, setup

package_name = 'omx_commander'

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
    maintainer='masutani',
    maintainer_email='masutani@isc.osakac.ac.jp',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topic_joint = omx_commander.topic_joint:main',
            'servo = omx_commander.servo:main',
            'moveit_joint = omx_commander.moveit_joint:main',
        ],
    },
)
