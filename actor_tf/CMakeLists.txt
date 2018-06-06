cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(actor_tf actor_tf.cc)
target_link_libraries(actor_tf ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} pthread tf)
