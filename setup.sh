#!/bin/bash

# Create a virtual environment
python3 -m venv .venv
source .venv/bin/activate
cd .venv
touch COLCON_IGNORE
cd ..

# Pull external python libraries source code and install
mkdir .lib_src
cd .lib_src
touch COLCON_IGNORE
git clone https://github.com/bitcraze/crazyflie-lib-python.git
cd crazyflie-lib-python
pip install -e .
cd ../..

# Install other python libraries
pip install motioncapture

# Colcon build
cd ../..
colcon build --base-path src/Crazyflie-P4-Group-460/
source install/local_setup.bash
cd src/Crazyflie-P4-Group-460/