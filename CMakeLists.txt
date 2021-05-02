cmake_minimum_required(VERSION 3.1)
project(mfplan LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Turn tests on with "cmake -Dtest=ON"
option(test "Build all tests" OFF)

find_package(ompl REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(LEMON REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${LEMON_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    include
)

add_library(dijkstra src/dijkstra.cc include/mfplan/dijkstra.h)

add_library(mfplanner src/mfplanner.cc include/mfplan/mfplanner.h)
target_link_libraries(mfplanner
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${OpenCV_LIBS}
    dijkstra
)

add_executable(run_planner src/run_planner.cc)
target_link_libraries(run_planner
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    dijkstra
    mfplanner
)

if (test)
  enable_testing()
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})

  add_executable(test_dijkstra tests/test_dijkstra.cc)
  target_link_libraries(test_dijkstra dijkstra GTest::Main GTest::GTest)
  add_test(NAME test COMMAND test_dijkstra)
endif (test)

install(TARGETS run_planner DESTINATION bin)
