# c++ code or something

Not ROS stuff yet.  Just trying to get up and running with as little setup as possible.

## Installation instructions

Make sure OMPL, Boost, and OpenCV are installed.  With homebrew:
```
brew install ompl
```

## Build instructions

From root directory:
```
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


