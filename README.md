# manif
## A small header-only library for Lie algebra.

## Package Summary
**manif** is a header-only c++11 Lie algebra library for state-estimation targeted at robotics applications.

-   Maintainer status: maintained
-   Maintainer: Jeremie Deray [deray.jeremie@gmail.com](mailto:deray.jeremie@gmail.com)
-   Author: Jeremie Deray [deray.jeremie@gmail.com](mailto:deray.jeremie@gmail.com)
-   License: APACHE-2.0
-   Bug / feature tracker: [github.com/artivis/manif/issues](https://github.com/artivis/manif/issues)
-   Source: [github.com/artivis/manif.git](https://github.com/artivis/manif.git) (branch: devel)

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
In your project `CMakeLists.txt` :
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
| Composition | <img src="https://latex.codecogs.com/png.latex?\mathbf&space;\mathcal{X}&space;\circ&space;\mathbf&space;\mathcal{Y}" title="\mathbf \mathcal{X} \circ \mathbf \mathcal{Y}" /> | `X * Y`<br/>`X.compose(Y)` |
| Retract to group element | <img src="https://latex.codecogs.com/png.latex?\exp(\mathbf\varphi)" title="\exp(\mathbf \varphi)" /> | `w.retract()` |
| Act on vector | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\circ\mathbf&space;v"/> | `X.act(v)` |
| Lift to tangent space | <img src="https://latex.codecogs.com/png.latex?\log(\mathbf&space;\mathcal{X})" title="\log(\mathbf \Phi)" /> | `X.lift()` |
| Manifold Adjoint | <img src="https://latex.codecogs.com/png.latex?Adj(\mathbf&space;\mathcal{X})" /> | `X.adj()` |
| Tangent adjoint | <img src="https://latex.codecogs.com/png.latex?adj(\mathbf&space;\varphi)" /> | `w.smallAdj()` |
|       |   Composed Operation   |  |
| Manifold right plus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\oplus\mathbf\varphi=\mathbf\mathcal{X}\circ\exp(\mathbf\varphi)" /> | `X + w`<br/>`X.plus(w)`<br/>`X.rplus(w)` |
| Manifold left plus | <img src="https://latex.codecogs.com/png.latex?\mathbf\varphi\oplus\mathbf\mathcal{X}=\exp(\mathbf\varphi)\circ\mathbf\mathcal{X}" /> | `w + X`<br/>`w.plus(X)`<br/>`w.lplus(X)` |
| Manifold right minus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\ominus\mathbf\mathcal{Y}=\log(\mathbf\mathcal{Y}^{-1}\circ\mathbf\mathcal{X})"  /> | `X - Y`<br/>`X.minus(Y)`<br/>`X.rminus(Y)` |
| Manifold left minus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{Y}\ominus\mathbf\mathcal{X}=\log(\mathbf\mathcal{X}\circ\mathbf\mathcal{Y}^{-1})"  /> | `X.lminus(Y)` |
| Between | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}^{-1}\circ\mathbf\mathcal{Y}"/> | `X.between(Y)` |

Above, <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X},\mathbf\mathcal{Y}" alt="\mathcal{Y}" /> represents a group element, <img src="https://latex.codecogs.com/png.latex?\mathbf\varphi" alt="small phi" />  or `w` represents an element of the tangent space and <img src="https://latex.codecogs.com/png.latex?\mathbf{v}" alt="v" /> or `v` represents any element of <img src="https://latex.codecogs.com/png.latex?\mathbb{R}^n" />.

### Jacobians

All operations come with their respectives analytical Jacobian matrices.  
Thoughout `manif`, **Jacobians are differentiated with respect to a local perturbation on the tangent space**.

<p align="center"><a href="https://www.codecogs.com/eqnedit.php?latex=\frac{\delta&space;f(\mathbf\mathcal{X})}{\delta\mathbf\mathcal{X}}\to\lim_{\varphi\to0}\frac{&space;f(\mathbf\mathcal{X}\oplus\varphi)\ominus&space;f(\mathbf\mathcal{X})}{\varphi}" target="_blank"><img src="https://latex.codecogs.com/svg.latex?\frac{\delta&space;f(\mathbf\mathcal{X})}{\delta\mathbf\mathcal{X}}\to\lim_{\varphi\to0}\frac{&space;f(\mathbf\mathcal{X}\oplus\varphi)\ominus&space;f(\mathbf\mathcal{X})}{\varphi}" title="\frac{\delta f(\mathbf\mathcal{X})}{\delta\mathbf\mathcal{X}}\to\lim_{\varphi\to0}\frac{ f(\mathbf\mathcal{X}\oplus\varphi)\ominus f(\mathbf\mathcal{X})}{\varphi}" /></a></center>&nbsp;

The Jacobians of any of the aforementionned operations can then be evaluated, e.g.,

```cpp
  SO2 X = SO2::Random(),
      Y = SO2::Random();

  SO2::Jacobian J_c_x, J_c_y;
  auto compose = x.compose(Y, J_c_x, J_c_y);

  SO2::Jacobian J_m_x, J_m_y;
  auto minus   = x.minus(Y, J_m_x, J_m_y);

  SO2::Jacobian J_i_x;
  auto inverse = x.inverse(J_i_x);

  // etc...
```

Shall you be interested only in a specific Jacobian, it can be retrieved without evaluating the other:

```cpp
  auto composition = x.compose(Y, J_c_x);
```
or conversely,
```cpp
  auto composition = x.compose(Y, SO2::_, J_c_y);
```

#### A note on Jacobians

While the `manif` package differentiates Jacobians wrt a perturbation on the tangent space, many non-linear solvers
(e.g. Ceres) expect them to be differentiated wrt the representation vector of the group element
(e.g. wrt to quaternion vector for <img src="https://latex.codecogs.com/png.latex?SO^3"/>).
For this reason `manif` is compliant with Ceres auto-differentiation and support `ceres::Jet` type.

## Documentation and Tutorials
@todo
<!--Some documentation that may point to a [wiki-page](https://github.com/artivis/manif/wiki/blablapage).  
Although I like packages using [readthedocs](https://readthedocs.org/) and [codedocs](https://codedocs.xyz/).-->

## Contributing
@todo  
These are the contribution guidelines.
-   `master` branch is for release only.
-   `devel` is the target for PR adding new features etc.

<!--## Credits
I wanna thanks my European project-->
