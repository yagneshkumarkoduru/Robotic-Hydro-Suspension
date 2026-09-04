from setuptools import setup, find_packages

setup(
    name='robotic-vehicle-project',
    version='0.1.0',
    description='Advanced Robotic Vehicle Control System',
    long_description=open('docs/README.md').read(),
    long_description_content_type='text/markdown',
    
    # Project metadata
    author='Ronin 1067',
    author_email='yagneshkumar.k23@iiits.in',
    url='https://github.com/Ronin1067/Robotic-Hydro-Suspension-Project',
    
    # Package discovery
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    
    # Dependencies
    install_requires=[
        'python-can>=4.1.0',
        'RPi.GPIO>=0.7.1',
        'smbus2>=0.4.2',
        'adafruit-circuitpython-vl53l0x>=5.1.10',
        'mpu6050-raspberrypi>=1.2.0',
        'numpy>=1.22.4',
        'pyserial>=3.5',
        'bluepy>=1.3.0',
    ],
    
    # Development dependencies
    extras_require={
        'dev': [
            'pytest>=7.3.1',
            'pylint>=2.17.4',
            'ipython>=8.13.2',
        ],
        'test': [
            'pytest>=7.3.1',
            'mock>=5.0.2',
        ],
    },
    
    # Classifier for project type and compatibility
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Topic :: Scientific/Engineering :: Robotics',
    ],
    
    # Entry points for command-line scripts
    entry_points={
        'console_scripts': [
            'robotic-vehicle=src.main:main',
        ],
    },
    
    # Additional project data
    python_requires='>=3.8',
    keywords='robotics vehicle control iot embedded-systems',
)