cmake helper for build muti library project

how to use:
1. edit CMakeLists.txt
```
set(LIB_SRC src/demolib2.cpp)
mc_add_library(demolib2 ${LIB_SRC})
target_link_libraries(demolib2 demolib1)
mc_install_library(demolib2)
```
2. compile
```
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../install"
make install
```
3. pack
```
cpack
```
4. find_package in other project
```
find_package(demo 0.0.0 REQUIRED)

message(STATUS "demo_INCLUDE_DIRS: " ${demo_INCLUDE_DIRS} )
message(STATUS "demo_LIBRARIES: " ${demo_LIBRARIES} )

add_executable(code main.cpp)
target_link_libraries(code  ${demo_LIBRARIES})
```

reference:
- https://foonathan.net/blog/2016/03/03/cmake-install.html
- https://github.com/pablospe/cmake-example-library

