#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

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
pip install catkin_pkg
pip install empy
pip install lark
pip install pyyaml
pip install bagpy

# Colcon build
cd ../..
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --base-path src/Crazyflie-P4-Group-460/
source install/setup.bash # This or local_setup.bash, i dont know the difference
cd src/Crazyflie-P4-Group-460/
