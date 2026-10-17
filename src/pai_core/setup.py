from setuptools import setup

setup(
    name='pai_core',
    version='0.1.0',
    packages=['pai_core'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/pai_core']),
                ('share/pai_core', ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
)
