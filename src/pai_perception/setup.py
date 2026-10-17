from setuptools import setup

setup(
    name='pai_perception',
    version='0.1.0',
    packages=['pai_perception'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/pai_perception']),
                ('share/pai_perception', ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'perception_node = pai_perception.perception_node:main',
        'perception_check = pai_perception.perception_check:main',
    ]},
)
