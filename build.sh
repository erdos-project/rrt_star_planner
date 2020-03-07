#!/bin/bash
sudo apt-get install build-essential
sudo apt-get install qtcreator
sudo apt-get install qt5-default
if [ ! -d "build" ]; then
  mkdir build
fi
if [ ! -d "bin" ]; then
    mkdir bin
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target all -- -j 8
