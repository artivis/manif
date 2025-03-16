# Getting started in C++

**manif** has been designed for an easy integration to larger projects,

- a single dependency on [Eigen][eigen]
- header-only
- templated on the underlying scalar type so that one can use its own
- and C++11, since not everyone gets to enjoy the latest version, especially in industry

## Installation

Prior to installing **manif**,
we have to install its main dependency `Eigen3`:

`````{tab-set}
````{tab-item} Ubuntu
:sync: key1
```bash
apt-get install libeigen3-dev
```
````
````{tab-item} OS X
:sync: key2
```bash
brew install eigen
```
````
`````

**manif** also depends on [lt::optional][optional-repo] however it is included in the [`external/`](https://github.com/artivis/manif/tree/devel/external) folder of the project.

Next, clone the repository locally if you haven't done so already:

```bash
git clone https://github.com/artivis/manif.git
cd manif
```

To buid and install **manif**,
use the following commands:

```bash
mkdir build && cd build
cmake ..
make install
```

## Use manif in a project

To use **manif** in a project,
edit the `CMakeLists.txt` and add the following directives to first find `Eigen3`:

```cmake
# Find the Eigen library
find_package(Eigen3 REQUIRED)

# Add eigen3 include directories to the target
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
```

Next, add the following directives to find `manif` and add its include directories to the compliation target:

```cmake
# Find the manif library
find_package(manif REQUIRED)

# Add manif include directories to the target
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${manif_INCLUDE_DIRS})
```

Now that the project is able to find `manif` and its headers,
it can be included in the code, for instance:

```cpp
#include <manif/manif.h>

...

auto state = manif::SE3d::Identity();

...

```

From there, have a look at the howtos for further informations
as well as the  [examples](../reference/examples-cpp.md) to find several self-contained and self-explained executables implementing some real problems.

[//]: # (URLs)

[eigen]: http://eigen.tuxfamily.org
[crtp]: https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
[optional-repo]: https://github.com/TartanLlama/optional
[ceres-jet]: http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets
