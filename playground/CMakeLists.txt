# PCL

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(surface_example example.cpp)
target_link_libraries (surface_example ${PCL_LIBRARIES})

add_executable(reader_sample reader.cpp)
target_link_libraries (reader_sample ${PCL_LIBRARIES})