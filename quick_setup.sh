#!/bin/bash

source .venv/bin/activate
cd ../..
colcon build --base-path src/Crazyflie-P4-Group-460/
source install/setup.bash # This or local_setup.bash, i dont know the difference
cd src/Crazyflie-P4-Group-460/
