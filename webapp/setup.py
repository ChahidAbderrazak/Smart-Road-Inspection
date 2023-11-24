from setuptools import setup

with open("README.md", "r", encoding="utf-8") as f:
    LONG_DESCRIPTION = f.read()

setup(
    name='HAIS-Highways-inspection',
    version='1.0',
    description='Inspect roads an highways infrastructure',
    long_description=LONG_DESCRIPTION,
    long_description_content_type='text/markdown',
    license='MIT',
    keywords='pytorch python inspection',
    author='Abderrazak Chahid',
    url='https://github.com/xxxxx',
    download_url='https://github.com/xxx/releases',
    install_requires=['numpy', 'Pillow', 'matplotlib'],
    packages=['RunHDD'],
    python_requires='>=3',
    zip_safe=False,
    entry_points={
        'console_scripts': [
            'RunHDD = __main__:main',
        ],
    },
    classifiers=[
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
    ],
)