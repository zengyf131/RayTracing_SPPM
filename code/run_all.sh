#!/usr/bin/env bash

# If project not ready, generate cmake file.
if [[ ! -d build ]]; then
    echo "good"
else
    rm -rf build
fi
mkdir -p build
cd build
cmake ..
make -j
cd ..

# Run all testcases. 
# You can comment some lines to disable the run of specific examples.
mkdir -p output
bin/FINAL testcases/rabbit_and_cup.txt output/rabbit_and_cup.bmp
# bin/FINAL testcases/sorry_rabbit.txt output/sorry_rabbit.bmp
# bin/FINAL testcases/CornellBox.txt output/CornellBox.bmp
# bin/FINAL testcases/CornellBox_cup.txt output/CornellBox_cup.bmp
# bin/FINAL testcases/textured_walls.txt output/textured_walls.bmp
# bin/FINAL testcases/scene01_basic.txt output/scene01.bmp
# bin/FINAL testcases/scene02_cube.txt output/scene02.bmp
# bin/FINAL testcases/scene06_bunny_1k.txt output/scene06.bmp