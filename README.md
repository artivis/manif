

# How to use

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(foo)

# Find the manif library
find_package(manif REQUIRED)

add_executable(${PROJECT_NAME} src/foo.cpp)

# Link against it.
# Include directories are automatically resolved
target_link_libraries(${PROJECT_NAME} ${manif_LIBRARIES})
```
