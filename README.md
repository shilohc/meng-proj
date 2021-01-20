# Code for MEng project

## Installation instructions

Make sure OMPL, Boost, and OpenCV are installed ([instructions](http://ompl.kavrakilab.org/installation.html) for installing OMPL).  

Build and install LEMON from source ([instructions](http://lemon.cs.elte.hu/trac/lemon/wiki/InstallLinux)).  This code has been tested only with version 1.3.1.  

## Build instructions

From root directory:
```
mkdir build && mkdir install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
make && make install
```

## Run instructions

From root directory:
```
./install/bin/rrt_test
./install/bin/lemon_test
```


