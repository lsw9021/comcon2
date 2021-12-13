cmake_minimum_required(VERSION 3.10.2)
project(com)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release
RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

add_compile_options(-fPIC)
add_compile_options(-Wdeprecated-declarations)

add_compile_options(-DHAVE_CSTDDEF)

add_definitions(-DROOT_DIR="${CMAKE_HOME_DIRECTORY}")

add_subdirectory( sim )
add_subdirectory( python )
#add_subdirectory( render )
