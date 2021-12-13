cmake_minimum_required(VERSION 3.10.2)
project(pycomcon)

link_directories(../sim/)
include_directories(../sim/)

add_compile_options(-fPIC)
add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)


include(FindOpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# find_package(TinyXML2 REQUIRED)
find_package(DART REQUIRED COMPONENTS collision-bullet CONFIG)
find_package(PythonLibs REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(${DART_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIR})
# file(GLOB srcs "Env.hpp")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
add_library(pycomcon SHARED "Env.h" "Env.cpp")
target_link_libraries(pycomcon ${DART_LIBRARIES} comcon ${PYTHON_LIBRARIES} pybind11::module)


set_target_properties(pycomcon PROPERTIES PREFIX "" )