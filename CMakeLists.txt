cmake_minimum_required(VERSION 3.1)
project(meng_proj)
set(CMAKE_CXX_STANDARD 17)

find_package(ompl REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(lemon REQUIRED)
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${LEMON_INCLUDE_DIRS}
)

add_executable(rrt_test rrt_test.cc)
target_link_libraries(rrt_test
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(lemon_test lemon_test.cc)

install(TARGETS rrt_test lemon_test DESTINATION bin)
