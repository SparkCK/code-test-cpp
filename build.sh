#!/bin/bash

rm -rfv build
mkdir build
cd build
cmake ..
make -j4
make install
cd ..