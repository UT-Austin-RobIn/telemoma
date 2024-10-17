# read the contents of your README file
from os import path
from setuptools import find_packages, setup

this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, "README.md"), encoding="utf-8") as f:
    lines = f.readlines()

# remove images from README
lines = [x for x in lines if ".png" not in x]
long_description = "".join(lines)


setup(
    name="telemoma",
    version="0.2.0",
    author="Shivin Dass",
    author_email='shivindass@gmail.com',
    description='A modular and versatile teleoperation system for mobile manipulation',
    long_description_content_type="text/markdown",
    long_description=long_description,
    url="https://github.com/UT-Austin-RobIn/telemoma",
    install_requires=[
        "numpy",
        "opencv-python",
        "mediapipe",
        "scipy",
        "pyrealsense2",
        "pyspacemouse",
        "pynput",
        "matplotlib",
        "gym",
    ],
    packages=find_packages(),
    python_requires=">=3",
) 
