# manif
## A small header-only library for Lie theory.

[![travis](https://travis-ci.com/artivis/manif.svg?branch=devel)](https://travis-ci.com/artivis/manif)
[![appveyor](https://ci.appveyor.com/api/projects/status/l0q7b0shhonvejrd?svg=true)](https://ci.appveyor.com/project/artivis/manif)
[![Documentation](https://codedocs.xyz/artivis/manif.svg)](https://codedocs.xyz/artivis/manif/)
[![codecov](https://codecov.io/gh/artivis/manif/branch/devel/graph/badge.svg)](https://codecov.io/gh/artivis/manif)
![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)
[![JOSS](http://joss.theoj.org/papers/e3fc778689407f0edd19df8c2089c160/status.svg)](http://joss.theoj.org/papers/e3fc778689407f0edd19df8c2089c160)

## Package Summary
**manif** is a header-only C++11 Lie theory library for state-estimation
targeted at robotics applications.

At the moment, it provides the groups:
  - R(n): Euclidean space with addition.
  - SO(2): rotations in the plane.
  - SE(2): rigid motion (rotation and translation) in the plane.
  - SO(3): rotations in 3D space.
  - SE(3): rigid motion (rotation and translation) in 3D space.
  - SE_2(3): extended pose (rotation, translation and velocity) in 3D space, introduced (to the best of knowledge) in this [paper](https://arxiv.org/pdf/1410.1465.pdf).  NOTE: The implementation here differs slightly from the developments in the [paper](https://arxiv.org/pdf/1410.1465.pdf).

Other Lie groups can and will be added, contributions are welcome.

**manif** is based on the mathematical presentation of the Lie theory available in [this paper](http://arxiv.org/abs/1812.01537).
We recommend every user of **manif** to read the paper (17 pages) before starting to use the library.
The paper provides a thorough introduction to Lie theory, in a simplified way so as to make the entrance to Lie theory easy for the average robotician who is interested in designing rigorous and elegant state estimation algorithms.
In a rush? Check out our [Lie group cheat sheet](paper/Lie_theory_cheat_sheet.pdf).

**manif** has been designed for an easy integration to larger projects:
  - A single dependency on [Eigen](http://eigen.tuxfamily.org),
  - header-only for easy integration,
  - templated on the underlying scalar type so that one can use its own,
  - and C++11, since not everyone gets to enjoy the latest C++ features, especially in industry.

It provides analytic computation of Jacobians for all the operations.
It also supports template scalar types. In particular, it can work with the
[`ceres::Jet`](http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets) type, allowing for automatic Jacobian computation --
[see related paragraph on Jacobians below](#jacobians).

All Lie group classes defined in **manif** have in common that they inherit from a templated base class ([CRTP](https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern)).
It allows one to write generic code abstracting the Lie group details.  
Please find more information in the related [wiki page](https://github.com/artivis/manif/wiki/Writing-generic-code)

#### Details

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
  <a href="#documentation">Documentation</a> •
  <a href="#tutorials-and-application-demos">Tutorials</a> •
  <a href="#publications">Publications</a> •
  <a href="#they-use-manif">They use manif</a> •
  <a href="#contributing">Contributing</a>
</p>
___

## Quick Start

### Dependencies

- Eigen 3 :
    + Linux ( Ubuntu and similar ) 
    
      ```terminal
      apt-get install libeigen3-dev
      ```
    
    + OS X 
    
      ```terminal
      brew install eigen
      ```

- [lt::optional](https://github.com/TartanLlama/optional) : included in the `external` folder

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
$ cmake -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON ..
$ make
$ make install
```

###### Using catkin_tools
```terminal
$ git clone https://github.com/artivis/manif.git
$ catkin build manif --cmake-args -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
```

###### Generate the documentation
```terminal
$ cd manif
$ doxygen .doxygen.txt
```

#### Use **manif** in your project
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

## Features

### Available Operations

| Operation  |       | Code |
| :---       |   :---:   | :---: |
|       |   Base Operation   |  |
| Inverse | <img src="https://latex.codecogs.com/png.latex?\mathbf&space;\mathcal{X}^{-1}" title="\mathbf \Phi^{-1}" /> | `X.inverse()` |
| Composition | <img src="https://latex.codecogs.com/png.latex?\mathbf&space;\mathcal{X}&space;\circ&space;\mathbf&space;\mathcal{Y}" title="\mathbf \mathcal{X} \circ \mathbf \mathcal{Y}" /> | `X * Y`<br/>`X.compose(Y)` |
| Hat | <img src="https://latex.codecogs.com/png.latex?\varphi^\wedge"/> | `w.hat()` |
| Act on vector | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\circ\mathbf&space;v"/> | `X.act(v)` |
| Retract to group element | <img src="https://latex.codecogs.com/png.latex?\exp(\mathbf\varphi^\wedge)" title="\exp(\mathbf \varphi^{^})" /> | `w.exp()` |
| Lift to tangent space | <img src="https://latex.codecogs.com/png.latex?\log(\mathbf&space;\mathcal{X})^\vee" title="\log(\mathbf \Phi)" /> | `X.log()` |
| Manifold Adjoint | <img src="https://latex.codecogs.com/png.latex?\operatorname{Adj}(\mathbf&space;\mathcal{X})" /> | `X.adj()` |
| Tangent adjoint | <img src="https://latex.codecogs.com/png.latex?\operatorname{adj}(\mathbf&space;\varphi^\wedge)" /> | `w.smallAdj()` |
|       |   Composed Operation   |  |
| Manifold right plus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\oplus\mathbf\varphi=\mathbf\mathcal{X}\circ\exp(\mathbf\varphi^\wedge)" /> | `X + w`<br/>`X.plus(w)`<br/>`X.rplus(w)` |
| Manifold left plus | <img src="https://latex.codecogs.com/png.latex?\mathbf\varphi\oplus\mathbf\mathcal{X}=\exp(\mathbf\varphi^\wedge)\circ\mathbf\mathcal{X}" /> | `w + X`<br/>`w.plus(X)`<br/>`w.lplus(X)` |
| Manifold right minus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\ominus\mathbf\mathcal{Y}=\log(\mathbf\mathcal{Y}^{-1}\circ\mathbf\mathcal{X})^\vee"  /> | `X - Y`<br/>`X.minus(Y)`<br/>`X.rminus(Y)` |
| Manifold left minus | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}\ominus\mathbf\mathcal{Y}=\log(\mathbf\mathcal{X}\circ\mathbf\mathcal{Y}^{-1})^\vee"  /> | `X.lminus(Y)` |
| Between | <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X}^{-1}\circ\mathbf\mathcal{Y}"/> | `X.between(Y)` |
| Inner Product | <img src="https://latex.codecogs.com/png.latex?\langle\varphi,\tau\rangle"/> | `w.inner(t)` |
| Norm | <img src="https://latex.codecogs.com/png.latex?\left\lVert\varphi\right\rVert"/> | `w.weightedNorm()`<br/>`w.squaredWeightedNorm()` |

Above, <img src="https://latex.codecogs.com/png.latex?\mathbf\mathcal{X},\mathbf\mathcal{Y}" alt="\mathcal{Y}" /> represent group elements, <img src="https://latex.codecogs.com/png.latex?\mathbf\varphi^\wedge,\tau^\wedge" alt="small phi" />  represent  elements in the Lie algebra of the Lie group, <img src="https://latex.codecogs.com/png.latex?\mathbf\varphi,\tau" alt="small phi" />  or `w,t` represent the same elements of the tangent space but expressed in Cartesian coordinates in  <img src="https://latex.codecogs.com/png.latex?\mathbb{R}^n" />, and <img src="https://latex.codecogs.com/png.latex?\mathbf{v}" alt="v" /> or `v` represents any element of <img src="https://latex.codecogs.com/png.latex?\mathbb{R}^n" />.

### Jacobians

All operations come with their respective analytical Jacobian matrices.  
Throughout **manif**, **Jacobians are differentiated with respect to a local perturbation on the tangent space**.

Currently, **manif** implements the **right Jacobian** (see [here](http://arxiv.org/abs/1812.01537) for reference), whose definition reads:

<p align="center"><a href="https://www.codecogs.com/eqnedit.php?latex=\frac{\delta&space;f(\mathbf\mathcal{X})}{\delta\mathbf\mathcal{X}}=\lim_{\varphi\to0}\frac{&space;f(\mathbf\mathcal{X}\oplus\varphi)\ominus&space;f(\mathbf\mathcal{X})}{\varphi}" target="_blank"><img src="https://latex.codecogs.com/svg.latex?\frac{\delta&space;f(\mathbf\mathcal{X})}{\delta\mathbf\mathcal{X}}\triangleq\lim_{\varphi\to0}\frac{&space;f(\mathbf\mathcal{X}\oplus\varphi)\ominus&space;f(\mathbf\mathcal{X})}{\varphi}\triangleq\lim_{\varphi\to0}\frac{\log(f(\mathbf\mathcal{X})^{-1}&space;f(\mathbf\mathcal{X}\exp(\varphi^\wedge)))^\vee}{\varphi}" title="\frac{\delta f(\mathbf\mathcal{X})}{\delta\mathbf\mathcal{X}}\to\lim_{\varphi\to0}\frac{ f(\mathbf\mathcal{X}\oplus\varphi)\ominus f(\mathbf\mathcal{X})}{\varphi}" /></a></center>&nbsp;

The Jacobians of any of the aforementionned operations can then be evaluated, e.g.,

```cpp
  SO2d X = SO2d::Random(),
       Y = SO2d::Random();

  SO2d::Jacobian J_c_x, J_c_y;
  auto compose = X.compose(Y, J_c_x, J_c_y);

  SO2d::Jacobian J_m_x, J_m_y;
  auto minus   = X.minus(Y, J_m_x, J_m_y);

  SO2d::Jacobian J_i_x;
  auto inverse = X.inverse(J_i_x);

  // etc...
```

Shall you be interested only in a specific Jacobian, it can be retrieved without evaluating the other:

```cpp
  auto composition = X.compose(Y, J_c_x);
```

or conversely,

```cpp
  auto composition = X.compose(Y, SO2d::_, J_c_y);
```

#### A note on Jacobians

The **manif** package differentiates Jacobians with respect to a
local perturbation on the tangent space.
These Jacobians map tangent spaces, as described in [this paper](http://arxiv.org/abs/1812.01537).

However, many non-linear solvers
(e.g. [Ceres](http://ceres-solver.org/)) expect functions to be differentiated with respect to the underlying
representation vector of the group element
(e.g. with respect to quaternion vector for <img src="https://latex.codecogs.com/png.latex?SO(3)"/>).

For this reason **manif** is compliant with [Ceres](http://ceres-solver.org/)
auto-differentiation and the
[`ceres::Jet`](http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets) type.  

For reference of the size of the Jacobians returned when using ```ceres::Jet```, **manif** implements rotations in the following way:
  - SO(2) and SE(2): as a complex number with `real = cos(theta)` and `imag = sin(theta)` values.
  - SO(3), SE(3) and SE_2(3): as a unit quaternion, using the underlying `Eigen::Quaternion` type.

Therefore, the respective Jacobian sizes using ```ceres::Jet``` are as follows:
  - SO(2) : size 2
  - SO(3) : size 4
  - SE(2) : size 4
  - SE(3) : size 7
  - SE_2(3): size 10

For more information, please refer to the [Ceres wiki page](https://github.com/artivis/manif/wiki/Using-manif-with-Ceres).

## Documentation

The API documentation can be found online at [codedocs.xyz/artivis/manif](https://codedocs.xyz/artivis/manif/).

Some more general documentation and tips on the use of the library is available on the [wiki-page](https://github.com/artivis/manif/wiki).
<!--Although I like packages using [readthedocs](https://readthedocs.org/) and [codedocs](https://codedocs.xyz/).-->

To generate the documentation on your machine, type in the terminal

```terminal
$ cd manif
$ doxygen .doxygen.txt
```

and find it at `manif/doc/html/index.html`.

Throughout the code documentation we refer to 'the paper' which you can
find in the section <a href="#publications">Publications</a>.

## Tutorials and application demos

We provide some self-contained and self-explained executables implementing some real problems.
Their source code is located in `manif/examples/`.
These demos are:

-   [`se2_localization.cpp`](examples/se2_localization.cpp): 2D robot localization based on fixed landmarks using SE2 as robot poses. This implements the example V.A in the paper.
-   [`se2_localization_ukfm.cpp`](examples/se2_localization_ukfm.cpp): 2D robot localization based on fixed landmarks using SE2 as robot poses. This implements the filter described in ['A Code for Unscented Kalman Filtering on Manifolds (UKF-M)'](https://arxiv.org/pdf/2002.00878.pdf), M. Brossard, A. Barrau and S. Bonnabel.
-   [`se3_localization.cpp`](examples/se3_localization.cpp): 3D robot localization based on fixed landmarks using SE3 as robot poses. This re-implements the example above but in 3D.
-   [`se2_sam.cpp`](examples/se2_sam.cpp): 2D smoothing and mapping (SAM) with simultaneous estimation of robot poses and landmark locations, based on SE2 robot poses. This implements a the example V.B in the paper.
-   [`se3_sam.cpp`](examples/se3_sam.cpp): 3D smoothing and mapping (SAM) with simultaneous estimation of robot poses and landmark locations, based on SE3 robot poses. This implements a 3D version of the example V.B in the paper.
-   [`se3_sam_selfcalib.cpp`](examples/se3_sam_selfcalib.cpp): 3D smoothing and mapping (SAM) with self-calibration, with simultaneous estimation of robot poses, landmark locations and sensor parameters, based on SE3 robot poses. This implements a 3D version of the example V.C in the paper.
-   [`se_2_3_localization.cpp`](examples/se_2_3_localization.cpp): A strap down IMU model based 3D robot localization, with measurements of fixed landmarks, using SE_2_3 as extended robot poses (translation, rotation and linear velocity). 

## Publications

If you use this software, please consider citing
[this paper](https://joss.theoj.org/papers/10.21105/joss.01371#) as follows:

```
@article{Deray-20-JOSS,
  doi = {10.21105/joss.01371},
  url = {https://doi.org/10.21105/joss.01371},
  year = {2020},
  publisher = {The Open Journal},
  volume = {5},
  number = {46},
  pages = {1371},
  author = {Jérémie Deray and Joan Solà},
  title = {Manif: A micro {L}ie theory library for state estimation in robotics applications},
  journal = {Journal of Open Source Software}
}

```

You can also consider citing [this paper](http://arxiv.org/abs/1812.01537) as follows:

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
Since this is a versioned work, please refer to [version 4, available here](http://arxiv.org/abs/1812.01537v4), of the paper when cross-referencing with the **manif** documentation.
This will give the appropriate equation numbers.

### Lie group cheat sheets

In a rush? Here is your Lie group theory take away:
[Lie group cheat sheet](paper/Lie_theory_cheat_sheet.pdf).

## They use **manif**

You may find [here](projects.md) a list of work and projects using **manif**.  
Your project is not listed? Let us know about it!

## Contributing

**manif** is developed according to Vincent Driessen's [Gitflow Workflow](http://nvie.com/posts/a-successful-git-branching-model/).
This means,
-   the `master` branch is for releases only.
-   development is done on feature branches.
-   finished features are integrated via PullRequests into the branch `devel`.

For a PullRequest to get merged into `devel`, it must pass
-   Review by one of the maintainers.
    +   Are the changes introduced in scope of **manif**?
    +   Is the documentation updated?
    +   Are enough reasonable tests added?
    +   Will these changes break the API?
    +   Do the new changes follow the current style of naming?
-   Compile / Test / Run on all target environments.
