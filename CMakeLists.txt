cmake_minimum_required(VERSION 3.0.2)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)
ADD_DEFINITIONS(-DEIGEN_NO_DEBUG)
include_directories(${EIGEN3_INCLUDE_DIR})
 
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

file(GLOB_RECURSE ERTTREE_SRC src/*.cpp)
include_directories(src)

add_library(ertconnect SHARED ${ERTTREE_SRC})
target_link_libraries(ertconnect ${OMPL_LIBRARIES})
 
add_executable(minimalistic_example minimalistic_example.cpp)
target_link_libraries(example ${OMPL_LIBRARIES} ertconnect)

add_executable(example updated_example.cpp)
target_link_libraries(example ${OMPL_LIBRARIES} ertconnect)
