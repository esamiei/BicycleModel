cmake_minimum_required(VERSION 3.16)
project(BM)

set(CMAKE_CXX_STANDARD 17)


include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/Eigen)
 

set(SOURCES src/main.cpp src/VDynamics.cpp)


add_executable(BM ${SOURCES})

find_package(Python3 COMPONENTS Development NumPy)
target_include_directories(BM PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_link_libraries(BM Python3::Python Python3::NumPy)
