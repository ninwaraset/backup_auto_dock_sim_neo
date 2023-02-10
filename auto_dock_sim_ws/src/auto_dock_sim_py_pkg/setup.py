from setuptools import setup

package_name = 'auto_dock_sim_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waraset',
    maintainer_email='waraset@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = auto_dock_sim_py_pkg.my_node:main',
            '01_pub = auto_dock_sim_py_pkg.01_pub:main'
        ],
    },
)
