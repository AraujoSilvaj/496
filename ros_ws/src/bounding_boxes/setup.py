from setuptools import find_packages, setup

package_name = 'bounding_boxes'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'boxes = bounding_boxes.boxes:main',
		'laser = bounding_boxes.get_box_height:main',
        ],
    },
)

# ldconfig /usr/lib/aarch64-linux-gnu/tegra/

