

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
# Find the Qt libraries for Qt Quick/QML
find_package(Qt5 REQUIRED Core Qml Quick Gui)
add_definitions(${Qt5Widgets_DEFINITIONS} ${QtQml_DEFINITIONS} ${${Qt5Quick_DEFINITIONS}})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
function(mc_add_qml_qrc arg)
    message("======= mc_add_qml_qrc ======= ")
    message("-- executable name: " ${ARGV0})  # 打印第一个参数里的所有内容
    message("-- QMLQRC : " ${ARGN})  # 打印第一个参数里的所有内容
    qt5_add_resources(QMLQRC ${ARGN})
    set(${ARGV0}_QMLQRC ${QMLQRC} PARENT_SCOPE)
endfunction()

#qt5_add_resources(QMLQRC ${QMLQRCPATH})
function(mc_link_qt5 arg)
    message("======= mc_link_qt5 ======= ")
    message("-- lib name: " ${ARGV0})  # 打印第一个参数里的所有内容
    target_link_libraries(${ARGV0} Qt5::Core Qt5::Qml Qt5::Quick)


endfunction()
function(mc_add_qml_executable arg)
    if (NOT ${ARGV0}_QMLQRC)
        message(FATAL_ERROR "${ARGV0}_QMLQRC NOT SET, You can not do this at all, CMake will exit.")
    endif ()
    message("======= mc_add_qml_executable ======= ")
    #    message("===INSTALL_CMAKE_DIR== " ${INSTALL_CMAKE_DIR})
    message("-- executable name: " ${ARGV0})  # 打印第一个参数里的所有内容
    message("-- executable SRC: " ${ARGN})  # 打印第一个参数里的所有内容
    add_executable(${ARGV0} ${ARGN} ${${ARGV0}_QMLQRC})

    target_include_directories(${ARGV0} PUBLIC
            $<BUILD_INTERFACE:${HEADER_PATH}> # for headers when building
            $<BUILD_INTERFACE:${Qt5Widgets_INCLUDE_DIRS}>
            $<BUILD_INTERFACE:${QtQml_INCLUDE_DIRS}>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # for config_impl.hpp when building
            $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}/${PROJECT_NAME}> # for client in install mode
            $<INSTALL_INTERFACE:${INSTALL_LIB_DIR}> # for config_impl.hpp in install mode
            )

    target_link_libraries(${ARGV0} Qt5::Core Qt5::Qml Qt5::Quick)


endfunction()