from setuptools import find_packages, setup

package_name = 'm0609'

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
    maintainer='kmins',
    maintainer_email='kmins@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'topic_pub = m0609.topic_publisher:main',
            'topic_sub = m0609.topic_subscriber:main',
            'service_server = m0609.service_server:main',
            'service_client = m0609.service_client:main',
        ],
    },
)
