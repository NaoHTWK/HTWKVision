#!/bin/sh

mkdir ../vision-build
cd ../vision-build
cmake -DBUILD_DEMO_APP=1  ../vision
make -j install
cd -
./bin/HTWKVisionTests ./images/

