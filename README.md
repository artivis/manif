
# manif
## A library for on-Manifold operations.

## Package Summary
**manif** is a header-only C++ library for operations on Manifold targeted at robotics applications.


- Maintainer status: maintained
- Maintainer: Jeremie Deray <deray.jeremie@gmail.com>
- Author: Jeremie Deray <deray.jeremie@gmail.com>
- License: APACHE-2.0
- Bug / feature tracker: https://github.com/artivis/manif/issues
- Source: git https://github.com/artivis/manif.git (branch: devel)

[![Build Status](https://travis-ci.com/artivis/manif.svg?branch=devel)](https://travis-ci.com/artivis/manif)
---

## Quick Start

### Installation
<!--
#### Binaries
```terminal
$ apt-get install ros-indigo-my-package
```
-->
#### From source
```terminal
$ git clone https://github.com/artivis/manif.git
$ cd manif && mkdir build && cd build
$ cmake ..
$ make
```
###### To build also examples/tests
```terminal
$ cmake -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON ..
$ make
```
###### Using catkin_tools
```terminal
$ git clone https://github.com/artivis/manif.git
$ catkin build manif --cmake-args -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
```

#### Use `manif` in your project
In your project `CMakeLists.txt` add :
```cmake
project(foo)

# Find the manif library
find_package(manif REQUIRED)

add_executable(${PROJECT_NAME} src/foo.cpp)

# Add include directories to the target
target_include_directories(${PROJECT_NAME} SYSTEM ${manif_INCLUDE_DIRS})
```

## Some more details about the package

### Available Operations

| Operation  |       | Code |
| :---       |   :---:   | :---: |
|       |   Base Operation   |  |
| Inverse | <img src="https://latex.codecogs.com/png.latex?\mathbf&space;\mathcal{X}^{-1}" title="\mathbf \Phi^{-1}" /> | `X.inverse()` |
| Composition | <img src="https://latex.codecogs.com/png.latex?\mathbf&space;\mathcal{X}&space;\circ&space;\mathbf&space;\mathcal{Y}" title="\mathbf \mathcal{X} \circ \mathbf \mathcal{Y}" /> | <ul class="list-unstyled"><li>`X * Y`</li><li>`X.compose(Y)`</li></ul> |
| Retract to manifold space | <img src="https://latex.codecogs.com/png.latex?\exp(\mathbf\varphi)" title="\exp(\mathbf \varphi)" /> | `w.retract()` |
| Act on vector | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\circ\mathbf&space;v"/> | `X.act(v)` |
| Lift to tangent space | <img src="https://latex.codecogs.com/png.latex?\log(\mathbf&space;\mathcal{X})" title="\log(\mathbf \Phi)" /> | `X.lift()` |
| Manifold Adjoint | <img src="https://latex.codecogs.com/png.latex?Adj(\mathbf&space;\mathcal{X})" /> | `X.adj()` |
| Tangent adjoint | <img src="https://latex.codecogs.com/png.latex?adj(\mathbf&space;\varphi)" /> | `w.adj()` |
|       |   Composed Operation   |  |
| Manifold right plus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\oplus\mathbf\varphi=\mathbf\mathcal{X}\circ\exp(\mathbf\varphi)" /> | <ul class="list-unstyled"><li>`X + w`</li><li>`X.plus(w)`</li><li>`X.rplus(w)`</li></ul> |
| Manifold left plus | <img src="https://latex.codecogs.com/png.latex?\mathbf\varphi\oplus\mathbf\mathcal{X}=\exp(\mathbf\varphi)\circ\mathbf\mathcal{X}" /> | <ul class="list-unstyled"><li>`w + X`</li><li>`w.plus(X)`</li><li>`w.lplus(X)`</li></ul> |
| Manifold right minus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\ominus\mathbf\mathcal{Y}=\log(\mathbf\mathcal{Y}^{-1}\circ\mathbf\mathcal{X})"  /> | <ul class="list-unstyled"><li>`X - Y`</li><li>`X.minus(Y)`</li><li>`X.rminus(Y)`</li></ul> |
| Manifold left minus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{Y}\ominus\mathbf\mathcal{X}=\log(\mathbf\mathcal{X}\circ\mathbf\mathcal{Y}^{-1})"  /> | `X.lminus(Y)` |
| Between | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}^{-1}\circ\mathbf\mathcal{Y}"/> | `X.compose(Y)` |

Above, <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X},\mathbf\mathcal{Y}" alt="\mathcal{Y}" /> represents a manifold element, <img src="https://latex.codecogs.com/png.latex?\mathbf\varphi" alt="small phi" />  or `w` represents an element of the tangent space and <img src="https://latex.codecogs.com/png.latex?\mathbf{v}" alt="v" /> or `v` represents any element of <img src="https://latex.codecogs.com/png.latex?\mathbb{R}^n" />.

## Documentation and Tutorials
@todo
<!--Some documentation that may point to a [wiki-page](https://github.com/artivis/manif/wiki/blablapage).  
Although I like packages using [readthedocs](https://readthedocs.org/) and [codedocs](https://codedocs.xyz/).-->

## Contributing
@todo  
These are the contribution guidelines.
- `master` branch is for release only.
- `devel` is the target for PR adding new features etc.

<!--## Credits
I wanna thanks my European project-->
