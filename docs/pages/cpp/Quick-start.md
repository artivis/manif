# Quick start

- [Quick start](#quick-start)
  - [Installation](#installation)
    - [Dependencies](#dependencies)
    - [From source](#from-source)
  - [Use manif in your project](#use-manif-in-your-project)
  - [Tutorials and application demos](#tutorials-and-application-demos)

**manif** has been designed for an easy integration to larger projects:

- A single dependency on [Eigen][eigen],
- header-only for easy integration,
- templated on the underlying scalar type so that one can use its own,
- and C++11, since not everyone gets to enjoy the latest C++ features, especially in industry.

All Lie group classes defined in **manif** have in common that they inherit from a templated base class ([CRTP][crtp]).
It allows one to write generic code abstracting the Lie group details.
Please find more information in the related [documentation page](Writing-generic-code).

The library supports template scalar types. In particular, it can work with the
[`ceres::Jet`][ceres-jet] type, allowing for automatic Jacobian computation --
[see related paragraph on Jacobians below](#jacobians).

## Installation

### Dependencies

- Eigen 3 :
  - Linux ( Ubuntu and similar )

      ```
      apt-get install libeigen3-dev
      ```

  - OS X

      ```
      brew install eigen
      ```

- [lt::optional][optional-repo] : included in the `external` folder

### From source

```terminal
git clone https://github.com/artivis/manif.git
cd manif && mkdir build && cd build
cmake ..
make install
```

## Use manif in your project

In your project `CMakeLists.txt` :

```cmake
project(foo)
# Find the Eigen library
find_package(Eigen3 REQUIRED)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
# Find the manif library
find_package(manif REQUIRED)
add_executable(${PROJECT_NAME} src/foo.cpp)
# Add manif include directories to the target
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${manif_INCLUDE_DIRS})
```

In your code:

```cpp
#include <manif/manif.h>

...

auto state = manif::SE3d::Identity();

...

```

## Tutorials and application demos

We provide some self-contained and self-explained executables implementing some real problems.
Their source code is located in `manif/examples/`.
These demos are:

- [`se2_localization.cpp`](examples/se2_localization.cpp): 2D robot localization based on fixed landmarks using SE2 as robot poses. This implements the example V.A in the paper.
- [`se3_localization.cpp`](examples/se3_localization.cpp): 3D robot localization based on fixed landmarks using SE3 as robot poses. This re-implements the example above but in 3D.
- [`se2_sam.cpp`](examples/se2_sam.cpp): 2D smoothing and mapping (SAM) with simultaneous estimation of robot poses and landmark locations, based on SE2 robot poses. This implements a the example V.B in the paper.
- [`se3_sam.cpp`](examples/se3_sam.cpp): 3D smoothing and mapping (SAM) with simultaneous estimation of robot poses and landmark locations, based on SE3 robot poses. This implements a 3D version of the example V.B in the paper.
- [`se3_sam_selfcalib.cpp`](examples/se3_sam_selfcalib.cpp): 3D smoothing and mapping (SAM) with self-calibration, with simultaneous estimation of robot poses, landmark locations and sensor parameters, based on SE3 robot poses. This implements a 3D version of the example V.C in the paper.
- [`se_2_3_localization.cpp`](examples/se_2_3_localization.cpp): A strap down IMU model based 3D robot localization, with measurements of fixed landmarks, using SE_2_3 as extended robot poses (translation, rotation and linear velocity).

To build the demos, simply pass the related flag to CMake,

```terminal
cmake -DBUILD_EXAMPLES=ON ..
make
cd examples
./se2_localization
```

[//]: # (URLs)

[eigen]: http://eigen.tuxfamily.org
[crtp]: https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
[optional-repo]: https://github.com/TartanLlama/optional
[ceres-jet]: http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets
