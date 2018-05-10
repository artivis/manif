# lspdlog - Lazy spdlog
============================================

## A simple wrapper package to enable cool logging for your project.

This package is nothing but a tiny hacky wrapper around [spdlog](https://github.com/gabime/spdlog) logging library.  
It has no other aim than enabling a bunch of macros to ease logging for your project.

For a more customizable logging please refer directly to [spdlog wiki](https://github.com/gabime/spdlog/wiki/1.-QuickStart).

##### Disclamer:

All credits go to Gabi Melman and [spdlog](https://github.com/gabime/spdlog) contributors, for this super fast C++ logging library.  
I have only over-engineered a tiny `cmake/cpp` layer on top of it.

## Requirements

-   c++11 compliant compiler.
-   internet connexion (spdlog is downloaded from [github](https://github.com/gabime/spdlog)).

## Install

1.  Simply clone or copy this package in your project directory:

    ```terminal
    $ cd ~/your_project/root_directory/
    ~/your_project/root_directory$ git clone https://github.com/artivis/lspdlog.git
    ```

    or else

    ```terminal
    $ cd ~/your_project/root_directory/
    ~/your_project/root_directory$ git submodule add https://github.com/artivis/lspdlog.git
    ```

2.  In your project `CMakeLists.txt`:

    ```cmake
    find_package(Threads REQUIRED)

    add_subdirectory(lspdlog)
    include_directories(${LSPDLOG_INCLUDE_DIRS})

    add_executable(my_project my_project.cpp)
    add_dependencies(my_project spdlog)
    target_link_libraries(my_project ${CMAKE_THREAD_LIBS_INIT})
    ```

3.  Compile your project.

4.  Nop that's it.

#### Enabling logging in file (Optional):

In order to enable a specific macro that logs in a file (see below), add to your project `CMakeLists.txt`:

```cmake
# Set 'ON' to enable 'TRACE' macro.
option(LSPDLOG_ENABLE_TRACE_LOGGING "Enable trace logging." ON)
  
# Set 'ON' to enable 'LOG' macro.
option(LSPDLOG_ENABLE_DATA_LOGGING  "Enable data logging."  ON)
  
find_package(Threads REQUIRED)
  
add_subdirectory(lspdlog)
include_directories(${LSPDLOG_INCLUDE_DIRS})
  
add_executable(my_project my_project.cpp)
add_dependencies(my_project spdlog)
target_link_libraries(my_project ${CMAKE_THREAD_LIBS_INIT})
```

## Now what ?

This package defines automatically the following macros:

```cpp
YOUR_PROJECT_NAME_INFO(...);
YOUR_PROJECT_NAME_WARN(...);
YOUR_PROJECT_NAME_DEBUG(...);
YOUR_PROJECT_NAME_ERROR(...);
```

#### Optionally:

```cpp
YOUR_PROJECT_NAME_TRACE(...);
YOUR_PROJECT_NAME_LOG(...);
YOUR_PROJECT_NAME_SCOPED_LOG(...);
```

### An example:

Given your project `CMakeLists.txt`:

```cmake
project(my_awesome_project)

# Set 'ON' to enable 'TRACE' macro.
option(LSPDLOG_ENABLE_TRACE_LOGGING "Enable trace logging." OFF)

# Set 'ON' to enable 'LOG' macro.
option(LSPDLOG_ENABLE_DATA_LOGGING  "Enable data logging."  OFF)

find_package(Threads REQUIRED)                         #<-- necessary

# lspdlog & spdlog require c++11                       #<-- necessary and
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")  #<-- automatically set

SET(CMAKE_BUILD_TYPE "DEBUG")

add_subdirectory(lspdlog)                    #<-- necessary
include_directories(${LSPDLOG_INCLUDE_DIRS}) #<-- necessary

add_executable(my_project my_project.cpp)
add_dependencies(my_project spdlog) #<-- necessary, waits for downloading spdlog
target_link_libraries(my_project ${CMAKE_THREAD_LIBS_INIT})  #<-- necessary
```

and your main file `my_project.cpp`:

```cpp
#include "lspdlog/logging.h"

int main()
{
  std::cout << "Hello world." << std::endl;

  MY_AWESOME_PROJECT_INFO("Yep ", "that is");
  MY_AWESOME_PROJECT_WARN("way\t", 2);
  MY_AWESOME_PROJECT_DEBUG("(I meant 'too'");
  MY_AWESOME_PROJECT_ERROR("easy");

  return 0;
}
```

the following gets printed in your terminal:

```terminal
Hello world.
[info] Yep that is
[warning] way   2
[debug] (I meant 'too')
[error] easy
```

Now if we re-compile in release:

```cmake
SET(CMAKE_BUILD_TYPE "RELEASE")
```

```terminal
Hello world.
[info] Yep that is
[warning] way   2
[error] easy
```

#### Data logging (optional):

Given that you have set `ON` the data logging option (see section `Install (optional)`), the following macro is enabled:

```cpp
MY_AWESOME_PROJECT_LOG("Some data ", 2, " save.");
```

This functionality creates a `.my_awesome_project` folder in the user `HOME` directory such as:

```terminal
~/.my_awesome_project$
```

where it will save all data passed to the `MY_AWESOME_PROJECT_LOG(...)` macro.  
In order to keep things a little ordered, each time you run your program, the macro also creates a sub-directory named after the current date (MM_DD_YY - month-day-year) within which it saves the log file named after the current time (HH_MM_SS - hour-minutes-seconds):

```terminal
$ cat ~/.my_awesome_project/11_04_16/11_25_47.log
[11:25:47.351422712] Some data 2 log.
```

This functionality can be enabled/disabled at runtime with the following macro:

```cpp
MY_AWESOME_PROJECT_ENABLE_DATA_LOG();
MY_AWESOME_PROJECT_LOG("will get logged.");

MY_AWESOME_PROJECT_DISABLE_DATA_LOG();
MY_AWESOME_PROJECT_LOG("will NOT get logged.");

{
  MY_AWESOME_PROJECT_SCOPED_ENABLE_LOG();
  MY_AWESOME_PROJECT_LOG("will get logged.");
}

MY_AWESOME_PROJECT_LOG("will NOT get logged.");
```

Notice that the `~/.my_awesome_project` directory and sub-directories are created (if they don't already exists) if the logging is enabled, either through `CMake` or manually.


# Todo

-   [ ] fix (do) install rules & `ExternalProject_Add`
-   [x] trace macro
-   [x] scoped log macro
-   [ ] critical macro
-   [ ] async logger
-   [ ] enable more customization
