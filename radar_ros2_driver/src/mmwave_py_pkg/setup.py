from setuptools import setup

package_name = 'mmwave_py_pkg'
submodules = 'mmwave_py_pkg/mmwave_submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nedwob',
    maintainer_email='nedwob@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"radar_node=mmwave_py_pkg.mmwave_node:main"
        ],
    },
)
