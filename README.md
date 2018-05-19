

# How to use

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(foo)

# Find the manif library
find_package(manif REQUIRED)

add_executable(${PROJECT_NAME} src/foo.cpp)

# Add include directories to the target
target_include_directories(${PROJECT_NAME} SYSTEM ${manif_INCLUDE_DIRS})
```
