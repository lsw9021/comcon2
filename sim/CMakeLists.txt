cmake_minimum_required(VERSION 3.10.2)
project(comcon)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel.")

set(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY})

add_compile_options(-fPIC)
#add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)


find_package(DART REQUIRED COMPONENTS collision-bullet CONFIG)
# find_package(PythonLibs 3.6 EXACT REQUIRED)
# find_package(PythonInterp 3.6 EXACT REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(${DART_INCLUDE_DIRS})
file(GLOB srcs "*.h" "*.cpp" "kinematics/*.h" "kinematics/*.cpp")

add_library(comcon ${srcs})
target_link_libraries(comcon ${DART_LIBRARIES})
