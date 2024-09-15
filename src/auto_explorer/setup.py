from setuptools import find_packages, setup

package_name = 'auto_explorer'

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
    maintainer='shun',
    maintainer_email='shun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_goals =auto_explorer.auto_explorer:main',
            'waypoint_follower = auto_explorer.waypoint_follower:main',
            'waypoint_explorer = auto_explorer.waypoint_explorer:main'
        ],
    },
)
