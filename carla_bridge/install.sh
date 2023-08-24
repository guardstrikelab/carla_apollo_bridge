#!/bin/bash

# Copy map files to /apollo/modules/map/data
cp -r map/. /apollo/modules/map/data

# Install requirements
pip3 install -r requirements.txt

# Set environment variables
if [ -f ~/.bashrc ] && ! grep -q 'export PYTHONPATH=$PYTHONPATH:/apollo/cyber' ~/.bashrc; then
    echo 'export PYTHONPATH=$PYTHONPATH:/apollo/cyber' >> ~/.bashrc
    echo 'export PYTHONPATH=$PYTHONPATH:/apollo/cyber/python' >> ~/.bashrc
    echo 'export PYTHONPATH=$PYTHONPATH:/apollo' >> ~/.bashrc
    echo 'export PYTHONPATH=$PYTHONPATH:/apollo/modules' >> ~/.bashrc
    echo 'export PYTHONPATH=$PYTHONPATH:/apollo/modules/carla_bridge/carla_api/carla-0.9.14-py3.7-linux-x86_64.egg' >> ~/.bashrc
    echo 'export PYTHONPATH=$PYTHONPATH:/apollo/bazel-bin' >> ~/.bashrc
fi
source ~/.bashrc