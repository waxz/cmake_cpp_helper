cmake_minimum_required(VERSION 2.8.3)
project(demo)

#==== load cmake
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Set variables
include(SetEnv)
include(SetBuildFlag)
include(FindPackageHelper)
include(CmakeUtils)


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
include(FindQt5Qml)

add_library(backend resources/backend.cpp)
target_include_directories(backend PUBLIC resources)
mc_link_qt5(backend)

set(SOURCE
        src/qtdemo.cpp
        )
mc_add_qml_qrc(helloworld resources/qml.qrc)

mc_add_qml_executable(helloworld ${SOURCE})
target_link_libraries(helloworld backend)
# Use the Qml/Quick modules from Qt 5.


# Install targets
include(InstallConfig)
# pack to deb
include(CpackConfig)
