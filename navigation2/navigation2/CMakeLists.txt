cmake_minimum_required(VERSION 3.5)
project(navigation2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

add_compile_options(-Wno-error)

ament_package()
