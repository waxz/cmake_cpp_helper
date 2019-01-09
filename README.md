cmake helper for build muti library project

how to use:
1. edit CMakeLists.txt
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


reference:
- https://foonathan.net/blog/2016/03/03/cmake-install.html
- https://github.com/pablospe/cmake-example-library

