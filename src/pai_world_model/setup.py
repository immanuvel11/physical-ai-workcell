from setuptools import setup

setup(
    name='pai_world_model',
    version='0.1.0',
    packages=['pai_world_model'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/pai_world_model']),
                ('share/pai_world_model', ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': ['world_model = pai_world_model.world_model_node:main']},
)
