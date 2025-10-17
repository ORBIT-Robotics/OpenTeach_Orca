from setuptools import setup, find_packages

setup(
    name='open-teach',
    version='1.0.0',
    packages=find_packages(),
    description='Open-Teach:VR Teleoperation for Robotic Manipulation',
    install_requires=[
        'hydra-core',
        'omegaconf',
        'numpy',
        'opencv-python',
        'pyzmq',
        'pyyaml',
        'matplotlib',
        'scipy',
        'pillow',
        'h5py',
        'pyrealsense2',
        'transforms3d',
        'blosc',
    ],
    python_requires='>=3.8',
)