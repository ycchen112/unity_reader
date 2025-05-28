from setuptools import find_packages, setup

package_name = 'reader_adb_json'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'geometry_msgs'],
    zip_safe=True,
    maintainer='yc',
    maintainer_email='yc@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "reader_adb_json_node = reader_adb_json.reader_adb_json:main"
        ],
    },
)
