from setuptools import find_packages, setup

package_name = 'nav4wbot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, '/launch', ['launch/navigation.launch.py']),
        ('share/' + package_name, '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name, '/maps', ['maps/test_map.yaml']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swashbuckler',
    maintainer_email='sohan104kuna@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
