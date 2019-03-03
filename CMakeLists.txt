cmake_minimum_required(VERSION 3.0)
project(demo)

#==== load cmake
# required for unit testing using CMake's ctest command
include(CTest)
enable_testing()

# required modules for our task
include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckIncludeFiles)
include(ExternalProject)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(MCMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
# Set variables
include(SetEnv)
include(SetBuildFlag)
#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx -mtune=haswell -fopt-info-vec-optimized -mavx -ftree-vectorize -msse2 -ffast-math -fopt-info-vec-optimized  -opt-report=5 -opt-report-file=stderr -opt-report-phase=all")

include(FindPackageHelper)
include(CmakeUtils)
include(OptimizeForArchitecture)


find_package(HMM 0.0.0 REQUIRED)
include_directories(${HMM_INCLUDE_DIRS})

find_package(TBB)
# 1 set Header path
set(HEADER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include)
# 2 create lib
set(LIB_SRC src/demolib1.cpp)
mc_add_library(demolib1 ${LIB_SRC})
mc_install_library(demolib1)

message(STATUS "PROJECT_LIBS: " ${PROJECT_LIBS})

set(LIB_SRC src/demolib2.cpp)
mc_add_library(demolib2 ${LIB_SRC})
target_link_libraries(demolib2 demolib1)
mc_install_library(demolib2)


message(STATUS "PROJECT_LIBS: " ${PROJECT_LIBS})
set(SRC src/demotest.cpp)
mc_add_executable(demo_test ${SRC})
target_link_libraries(demo_test demolib2)
mc_install_executable(demo_test)


## qt
#set(QMLQRCPATH resources/qml.qrc)
#include(FindQt5Qml)

add_library(backend include/cpp_utils/ros_util_impl.cpp
        include/cpp_utils/eigen_util_impl.cpp
        include/cpp_utils/ublas_impl.cpp
        )
target_include_directories(backend PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/resources> # for headers when building
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/resources> # for config_impl.hpp when building
        $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}/${PROJECT_NAME}> # for client in install mode
        $<INSTALL_INTERFACE:${INSTALL_LIB_DIR}> # for config_impl.hpp in install mode
        )
#target_include_directories(backend PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/resources)
#mc_link_qt5(backend)
mc_install_library(backend)


set(SOURCE
        src/main.cpp
        src/grid.cpp
        src/partial.cpp
        )
#mc_add_qml_qrc(helloworld resources/qml.qrc)


include(FindRosHelper)

mc_add_executable(helloworld ${SOURCE})
mc_include_dir(helloworld include)

target_link_libraries(helloworld backend)
# Use the Qml/Quick modules from Qt 5.
mc_install_executable(helloworld)
target_link_libraries(helloworld ${HMM_LIBRARIES} ${MultiPrecisionLib} ${ROS_LIBRARIES} ${CV_LIBS} ${TBB_LIBRARIES} ${Boost_LIBRARIES})


# Install targets
include(InstallConfig)
# pack to deb
include(CpackConfig)
