# manif
## A small header-only library for Lie theory. <!-- JS: why small? -->

[![Build Status](https://travis-ci.com/artivis/manif.svg?branch=devel)](https://travis-ci.com/artivis/manif)
![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)
<!-- ![](https://tokei.rs/b1/github/artivis/manif) -->

## Package Summary
**manif** is a header-only c++11 Lie theory library for state-estimation
targeted at robotics applications.

- Maintainer status: maintained
- Maintainer: Jeremie Deray [deray.jeremie@gmail.com](mailto:deray.jeremie@gmail.com)
- Authors:
  - Jeremie Deray [deray.jeremie@gmail.com](mailto:deray.jeremie@gmail.com)
  - Joan Sola [jsola@iri.upc.edu](mailto:jsola@iri.upc.edu)
- License: MIT
- Bug / feature tracker: [github.com/artivis/manif/issues](https://github.com/artivis/manif/issues)
- Source: [github.com/artivis/manif.git](https://github.com/artivis/manif.git) (branch: devel)

___

<p align="center">
  <a href="#installation">Installation</a> •
  <a href="#features">Features</a> •
  <!--a href="#examples">Examples</a> • -->
  <a href="#documentation">Documentation</a> •
  <a href="#tutorials-and-application-demos">Tutorials</a> •
  <a href="#publications">Publications</a> •
  <a href="#contributing">Contributing</a> <!--•
  <a href="#authors">Authors</a> -->
</p>

___

## Quick Start

### Dependencies

-   Eigen 3 : ```apt-get install libeigen3-dev```
-   [lt::optional](https://github.com/TartanLlama/optional) : included in the `external` folder

### Installation
<!--
#### Binaries
```terminal
$ apt-get install manif
```
-->
<!--#### From source-->

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

###### Generate the documentation
```terminal
cd [manif]
doxygen .doxygen.txt
```

#### Use `manif` in your project
In your project `CMakeLists.txt` :

```cmake
project(foo)
# Find the manif library
find_package(manif REQUIRED)
add_executable(${PROJECT_NAME} src/foo.cpp)
# Add manif include directories to the target
target_include_directories(${PROJECT_NAME} SYSTEM ${manif_INCLUDE_DIRS})
```

## Features

### Available Operations

| Operation  |       | Code |
| :---       |   :---:   | :---: |
|       |   Base Operation   |  |
| Inverse | <img src="https://latex.codecogs.com/png.latex?\mathbf&space;\mathcal{X}^{-1}" title="\mathbf \Phi^{-1}" /> | `X.inverse()` |
| Composition | <img src="https://latex.codecogs.com/png.latex?\mathbf&space;\mathcal{X}&space;\circ&space;\mathbf&space;\mathcal{Y}" title="\mathbf \mathcal{X} \circ \mathbf \mathcal{Y}" /> | `X * Y`<br/>`X.compose(Y)` |
| Hat | <img src="https://latex.codecogs.com/png.latex?\varphi^\wedge"/> | `w.hat()` |
| Retract to group element | <img src="https://latex.codecogs.com/png.latex?\exp(\mathbf\varphi^\wedge)" title="\exp(\mathbf \varphi^{^})" /> | `w.retract()` |
| Act on vector | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\circ\mathbf&space;v"/> | `X.act(v)` |
| Lift to tangent space | <img src="https://latex.codecogs.com/png.latex?\log(\mathbf&space;\mathcal{X})^\vee" title="\log(\mathbf \Phi)" /> | `X.lift()` |
| Manifold Adjoint | <img src="https://latex.codecogs.com/png.latex?Adj(\mathbf&space;\mathcal{X})" /> | `X.adj()` |
| Tangent adjoint | <img src="https://latex.codecogs.com/png.latex?adj(\mathbf&space;\varphi^\wedge)" /> | `w.smallAdj()` |
|       |   Composed Operation   |  |
| Manifold right plus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\oplus\mathbf\varphi=\mathbf\mathcal{X}\circ\exp(\mathbf\varphi^\wedge)" /> | `X + w`<br/>`X.plus(w)`<br/>`X.rplus(w)` |
| Manifold left plus | <img src="https://latex.codecogs.com/png.latex?\mathbf\varphi\oplus\mathbf\mathcal{X}=\exp(\mathbf\varphi^\wedge)\circ\mathbf\mathcal{X}" /> | `w + X`<br/>`w.plus(X)`<br/>`w.lplus(X)` |
| Manifold right minus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\ominus\mathbf\mathcal{Y}=\log(\mathbf\mathcal{Y}^{-1}\circ\mathbf\mathcal{X})^\vee"  /> | `X - Y`<br/>`X.minus(Y)`<br/>`X.rminus(Y)` |
| Manifold left minus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\ominus\mathbf\mathcal{Y}=\log(\mathbf\mathcal{X}\circ\mathbf\mathcal{Y}^{-1})^\vee"  /> | <br/>`X.lminus(Y)` |
| Between | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}^{-1}\circ\mathbf\mathcal{Y}"/> | `X.between(Y)` |
| Inner Product | <img src="https://latex.codecogs.com/png.latex?\langle \varphi,\varphi\rangle"/> | `w.inner(w)` |
| Norm | <img src="https://latex.codecogs.com/png.latex?\left\lVert\varphi\right\rVert"/> | `w.weightedNorm()`<br/>`w.squaredWeightedNorm()` |

Above, <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X},\mathbf\mathcal{Y}" alt="\mathcal{Y}" /> represent group elements, <img src="https://latex.codecogs.com/png.latex?\mathbf\varphi^\wedge" alt="small phi" />  represents an element in the Lie algebra of the Lie group, <img src="https://latex.codecogs.com/png.latex?\mathbf\varphi" alt="small phi" />  or `w` represents the same element of the tangent space but expressed in Cartesian coordinates in  <img src="https://latex.codecogs.com/png.latex?\mathbb{R}^n" />, and <img src="https://latex.codecogs.com/png.latex?\mathbf{v}" alt="v" /> or `v` represents any element of <img src="https://latex.codecogs.com/png.latex?\mathbb{R}^n" />.

### Jacobians

All operations come with their respective analytical Jacobian matrices.  
Throughout `manif`, **Jacobians are differentiated with respect to a local perturbation on the tangent space**.

<p align="center"><a href="https://www.codecogs.com/eqnedit.php?latex=\frac{\delta&space;f(\mathbf\mathcal{X})}{\delta\mathbf\mathcal{X}}=\lim_{\varphi\to0}\frac{&space;f(\mathbf\mathcal{X}\oplus\varphi)\ominus&space;f(\mathbf\mathcal{X})}{\varphi}" target="_blank"><img src="https://latex.codecogs.com/svg.latex?\frac{\delta&space;f(\mathbf\mathcal{X})}{\delta\mathbf\mathcal{X}}=\lim_{\varphi\to0}\frac{&space;f(\mathbf\mathcal{X}\oplus\varphi)\ominus&space;f(\mathbf\mathcal{X})}{\varphi}" title="\frac{\delta f(\mathbf\mathcal{X})}{\delta\mathbf\mathcal{X}}\to\lim_{\varphi\to0}\frac{ f(\mathbf\mathcal{X}\oplus\varphi)\ominus f(\mathbf\mathcal{X})}{\varphi}" /></a></center>&nbsp;

The Jacobians of any of the aforementionned operations can then be evaluated, e.g.,

```cpp
  SO2 X = SO2::Random(),
      Y = SO2::Random();

  SO2::Jacobian J_c_x, J_c_y;
  auto compose = X.compose(Y, J_c_x, J_c_y);

  SO2::Jacobian J_m_x, J_m_y;
  auto minus   = X.minus(Y, J_m_x, J_m_y);

  SO2::Jacobian J_i_x;
  auto inverse = X.inverse(J_i_x);

  // etc...
```

Shall you be interested only in a specific Jacobian, it can be retrieved without evaluating the other:

```cpp
  auto composition = X.compose(Y, J_c_x);
```

or conversely,

```cpp
  auto composition = X.compose(Y, SO2::_, J_c_y);
```

#### A note on Jacobians

While the `manif` package differentiates Jacobians with respect to a
local perturbation on the tangent space, many non-linear solvers
(e.g. [Ceres](http://ceres-solver.org/)) expect them to be differentiated wrt the underlying
representation vector of the group element
(e.g. wrt to quaternion vector for <img src="https://latex.codecogs.com/png.latex?SO^3"/>).
For this reason `manif` is compliant with [Ceres](http://ceres-solver.org/)
auto-differentiation and the
[`ceres::Jet`](http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets) type.

## Documentation

Some general documentation on the use of the library is available on the [wiki-page](https://github.com/artivis/manif/wiki).
<!--Although I like packages using [readthedocs](https://readthedocs.org/) and [codedocs](https://codedocs.xyz/).-->

Type in the terminal

```terminal
cd [manif]
doxygen .doxygen.txt
```

to obtain the code documentation. Find it at `[manif]/doc/html/index.html`.

Throughout the code documentation we refer to 'the paper' which you can
find in the section <a href="#publications">Publications</a>.

## Tutorials and application demos

We provide some self-contained and self-explained executables implementing some real problems.
Their source code is located in `[manif]/examples/`.
These demos are:

-   [`se2_localization.cpp`](examples/se2_localization.cpp): 2D robot localization based on fixed landmarks using SE2 as robot poses. This implements the example V.A in the paper.
-   [`se3_localization.cpp`](examples/se3_localization.cpp): 3D robot localization based on fixed landmarks using SE3 as robot poses. This re-implements the example above but in 3D.
-   [`se3_sam.cpp`](examples/se3_sam.cpp): 3D smoothing and mapping (SAM) with simultaneous estimation of robot poses and landmark locations, based on SE3 robot poses. This implements a 3D version of the example V.B in the paper.

## Publications

If you use this work, please consider citing [this paper](http://arxiv.org/abs/1812.01537) as follows:

```
@techreport{SOLA-18-Lie,
    Address = {Barcelona},
    Author = {Joan Sol\`a and Jeremie Deray and Dinesh Atchuthan},
    Institution = {{Institut de Rob\`otica i Inform\`atica Industrial}},
    Number = {IRI-TR-18-01},
    Title = {A micro {L}ie theory for state estimation in robotics},
    Howpublished="\url{http://arxiv.org/abs/1812.01537}",
    Year = {2018}
}
```
Notice that this reference is the one referred to throughout the code documentation.
Since this is a versioned work, please refer to [version 4, available here](http://arxiv.org/abs/1812.01537v4), of the paper when cross-referencing with the `manif` documentation.
This will give the appropriate equation numbers.

## Contributing

`manif` is developed according to Vincent Driessen's [Gitflow Workflow](http://nvie.com/posts/a-successful-git-branching-model/).
This means,
-   the master branch is for releases only.
-   development is done on feature branches.
-   finished features are integrated via PullRequests into develop.

For a PullRequest to get merged into develop, it must pass
-   Review by one of the maintainers.
    +   Are the changes introduces in scope of the `manif`?
    +   Is the documentation updated?
    +   Are enough reasonable tests added?
    +   Will these changes break the API?
    +   Do the new changes follow the current style of naming?
-   Compile / Test / Run on all target environments.

<!--## Credits
I wanna thank my European project-->
