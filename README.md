# Code for MEng project

## Installation instructions

This project has been tested on Ubuntu 18.04 LTS.  Parts of it will also work on MacOS Catalina, but OMPL seems to be hit or miss on MacOS.  Trying to get it to work on Windows is not recommended.

Make sure OMPL, Boost, and OpenCV are installed ([instructions](http://ompl.kavrakilab.org/installation.html) for installing OMPL).  

Build and install LEMON from source ([instructions](http://lemon.cs.elte.hu/trac/lemon/wiki/InstallLinux)).  This code has been tested only with version 1.3.1.  

Build and install [yaml-cpp](https://github.com/jbeder/yaml-cpp) from source:
```
git clone https://github.com/jbeder/yaml-cpp.git && cd yaml-cpp
mkdir build && cd build
cmake .. -DYAML_BUILD_SHARED_LIBS=ON
make install
```

If using tests, also build and install googletest from source:
```
git clone https://github.com/google/googletest.git && cd googletest
mkdir build && cd build
cmake ..
make && make install
```

Make sure to run `ldconfig` after all libraries have been installed:
```
sudo ldconfig
```

## Build instructions

From root directory:
```
mkdir build && mkdir install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
make && make install
```

If building with tests, use `cmake .. -Dtest=ON`.  (Requires installation of googletest.)

The error
```
fatal error: Eigen/Core: No such file or directory
```
is caused by OMPL expecting `Eigen`, but recent versions are named `eigen3`.  This can be fixed in a hacky way using a symlink:
```
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

## Run instructions

The primary executable `lemon_test` takes as argument YAML file describing a planning problem.  For example, running
```
./install/bin/lemon_test "problems/mit_basement_1_to_e19.yaml"
```
from the root directory of the repo will try to find a path from building 1 to building E19 in the MIT basement map.

## Test instructions

From the repo's root directory:
```
./build/test_dijkstra
```

## `parse_points.py`

This is a simple test script that will try to read `out.txt`, parse each line as a tuple `(x, y)`, and visualize these points as pink dots added to `test_out.png` (output to `test_out_with_points.png`).  Useful as a visualization tool with roughly the following workflow:
```
./install/bin/lemon_test > out.txt
[delete any lines in out.txt that aren't tuples (x, y)]
python3 parse_points.py
```

Requires `opencv-python` (install with pip).

