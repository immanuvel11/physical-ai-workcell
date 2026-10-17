from setuptools import setup

setup(
    name='pai_benchmark',
    version='0.1.0',
    packages=['pai_benchmark'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/pai_benchmark']),
                ('share/pai_benchmark', ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'system_check = pai_benchmark.system_check:main',
        'baseline = pai_benchmark.baseline:main',
        'benchmark = pai_benchmark.benchmark:main',
        'report = pai_benchmark.report:main',
    ]},
)
