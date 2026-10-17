from setuptools import setup

setup(
    name='pai_ui',
    version='0.1.0',
    packages=['pai_ui'],
    package_data={'pai_ui': ['index.html']},
    data_files=[('share/ament_index/resource_index/packages', ['resource/pai_ui']),
                ('share/pai_ui', ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=False,
    entry_points={'console_scripts': ['console = pai_ui.console:main']},
)
