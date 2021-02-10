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

## `parse_points.py`

This is a simple test script that will try to read `out.txt`, parse each line as a tuple `(x, y)`, and visualize these points as pink dots added to `test_out.png` (output to `test_out_with_points.png`).  Useful as a visualization tool with roughly the following workflow:
```
./install/bin/lemon_test > out.txt
[delete any lines in out.txt that aren't tuples (x, y)]
python3 parse_points.py
```

Requires `opencv-python` (install with pip).

