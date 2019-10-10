import setuptools 

with open("README.md","r") as fh:
      long_description = fh.read()

setuptools.setup(name='yrk',
      version='0.0.1',
      description='York Robotics Kit',
      url='http://www.york.ac.uk/robot-lab/yrk',
      author='James Hilder',
      author_email='james.hilder@york.ac.uk',
      license='MIT',
      packages=setuptools.find_packages(),
      classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
      ],
      python_requires='>=3.6',
)
