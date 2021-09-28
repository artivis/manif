# manif

## A small header-only library for Lie theory

[![GHA][badge-ci-img]][badge-ci]
[![appveyor][badge-ci-win-img]][badge-ci-win]
[![Documentation][badge-doc-img]][manif-doc]
[![codecov][badge-cov-img]][badge-cov]
![GitHub][badge-license]
[![JOSS][badge-joss-img]][deray20]

## Package Summary

**manif** is a Lie theory library for state-estimation
targeted at robotics applications.
It is developed as a header-only C++11 library with Python 3 wrappers.

At the moment, it provides the groups:

- ℝ(n): Euclidean space with addition.
- SO(2): rotations in the plane.
- SE(2): rigid motion (rotation and translation) in the plane.
- SO(3): rotations in 3D space.
- SE(3): rigid motion (rotation and translation) in 3D space.
- SE_2(3): extended pose (rotation, translation and velocity) in 3D space,
  introduced (to the best of knowledge) in this [paper][barrau15].
  NOTE: The implementation here differs slightly from
  the developments in the [paper][barrau15].
- Bundle<>: allows manipulating a manifold bundle as a single Lie group.
  Referred to as a *composite manifold* in Section IV of the
  [reference paper](http://arxiv.org/abs/1812.01537).

Other Lie groups can and will be added, contributions are welcome.

**manif** is based on the mathematical presentation of the Lie theory available in [this paper][jsola18].
We recommend every user of **manif** to read the paper (17 pages) before starting to use the library.
The paper provides a thorough introduction to Lie theory,
in a simplified way so as to make the entrance to Lie theory easy for the average roboticist
who is interested in designing rigorous and elegant state estimation algorithms.

You may also find the following video online,
['Lie theory for the roboticist'][jsola-iri-lecture],
a lecture given at [IRI-UPC][IRI-UPC].
In a rush? Check out our [Lie group cheat sheet][cheat_sheet].

It provides analytic computation of Jacobians for all the operations listed [below](#features).

### Details

- Maintainer status: maintained
- Maintainer: Jeremie Deray
- Authors:
  - Jeremie Deray [deray.jeremie@gmail.com](mailto:deray.jeremie@gmail.com)
  - Joan Sola [jsola@iri.upc.edu](mailto:jsola@iri.upc.edu)
- License: [MIT](LICENSE)
- Bug / feature tracker: [github.com/artivis/manif/issues][manif-issue]
- Source: [github.com/artivis/manif.git][manif-repo] (branch: devel)

## Quick Start

Get quickly started with **manif** following our 'quick start' guides for both
[C++](docs/pages/cpp/Quick-start.md) and [Python](docs/pages/python/Quick-start.md).

## Features

### Available Operations

| Operation  |       | Code |
| :---       |   :---:   | :---: |
|       |   Base Operation   |  |
| Inverse | ![\mathbf\Phi^{-1}][latex1] | `X.inverse()` |
| Composition | ![\mathbf\mathcal{X}\circ\mathbf\mathcal{Y}][latex2] | `X * Y`<br/>`X.compose(Y)` |
| Hat | ![\varphi^\wedge][latex3] | `w.hat()` |
| Act on vector | ![\mathbf\mathcal{X}\circ\mathbf v][latex4] | `X.act(v)` |
| Retract to group element | ![\exp(\mathbf\varphi^\wedge][latex5] | `w.exp()` |
| Lift to tangent space | ![\log(\mathbf\mathcal{X})^\vee][latex6] | `X.log()` |
| Manifold Adjoint | ![\operatorname{Adj}(\mathbf\mathcal{X})][latex7] | `X.adj()` |
| Tangent adjoint | ![\operatorname{adj}(\mathbf\varphi^\wedge][latex8] | `w.smallAdj()` |
|       |   Composed Operation   |  |
| Manifold right plus | ![\mathbf\mathcal{X}\circ\exp(\mathbf\varphi^\wedge)][latex9] | `X + w`<br/>`X.plus(w)`<br/>`X.rplus(w)` |
| Manifold left plus | ![\exp(\mathbf\varphi^\wedge)\circ\mathbf\mathcal{X}][latex10] | `w + X`<br/>`w.plus(X)`<br/>`w.lplus(X)` |
| Manifold right minus | ![\log(\mathbf\mathcal{Y}^{-1}\circ\mathbf\mathcal{X})^\vee][latex11] | `X - Y`<br/>`X.minus(Y)`<br/>`X.rminus(Y)` |
| Manifold left minus | ![\log(\mathbf\mathcal{X}\circ\mathbf\mathcal{Y}^{-1})^\vee][latex12] | `X.lminus(Y)` |
| Between | ![\mathbf\mathcal{X}^{-1}\circ\mathbf\mathcal{Y}][latex13] | `X.between(Y)` |
| Inner Product | ![\langle\varphi,\tau\rangle][latex14] | `w.inner(t)` |
| Norm | ![\left\lVert\varphi\right\rVert][latex15] | `w.weightedNorm()`<br/>`w.squaredWeightedNorm()` |

Above, ![\mathbf\mathcal{X},\mathbf\mathcal{Y}][latex16] represent group elements,
![\mathbf\varphi^\wedge,\tau^\wedge][latex17] represent elements in the Lie algebra of the Lie group,
![\mathbf\varphi,\tau][latex18] or `w,t` represent the same elements of the tangent space
but expressed in Cartesian coordinates in ![\mathbb{R}^n][latex19],
and ![\mathbf{v}][latex20] or `v` represents any element of ![\mathbb{R}^n][latex21].

### Jacobians

All operations come with their respective analytical Jacobian matrices.
Throughout **manif**, **Jacobians are differentiated with respect to a local perturbation on the tangent space**.
These Jacobians map tangent spaces, as described in [this paper][jsola18].

Currently, **manif** implements the **right Jacobian**, whose definition reads:

![\frac{\delta f(\mathbf\mathcal{X})}{\delta\mathbf\mathcal{X}}][latex22]

The Jacobians of any of the aforementionned operations can then be evaluated:

in C++,

```cpp
  SE3d X = SE3d::Random();
  SE3Tangentd w = SE3Tangentd::Random();

  SE3d::Jacobian J_o_x, J_o_w;

  auto X_plus_w = X.plus(w, J_o_x, J_o_w);
```

in Python,

```python
  X = SE3.Random()
  w = SE3Tangentd.Random()

  J_o_x = np.zeros((SE3.DoF, SE3.DoF))
  J_o_w = np.zeros((SE3.DoF, SE3.DoF))

  X_plus_w = X.plus(w, J_o_x, J_o_w)
```

#### Note

While Jacobians in **manif** are differentiated with respect to a
local perturbation on the tangent space, many non-linear solvers
(e.g. [Ceres][ceres]) expect functions to be differentiated with respect to the underlying representation vector of the group element
(e.g. with respect to quaternion vector for `SO3`).

For this reason, **manif** is compliant with the common auto-differentiation types
[`ceres::Jet`][ceres-jet],
[`autodiff::Dual`][autodiff] and [`CppAD::AD<Scalar>`][cppad] (forward/reverse).

## Documentation

The documentation is available online at the accompanying [website][manif-doc].
Both the [C++][manif-doc-cpp] and the [Python][manif-doc-python] APIs are documented.

Do you want to build it locally?
Find out how on the [dedicated page](docs/pages/documentation.md).

Note: throughout the code documentation we refer to 'the paper' which you can
find on [the dedicated page](docs/pages/publication.md).

## Tutorials and application demos

We provide some self-contained and self-explained [C++ examples](docs/pages/cpp/Quick-start.md#tutorials-and-application-demos) to help you get started.

You prefer Python? The same examples are also [available in Python](docs/pages/python/Quick-start.md#tutorials-and-application-demos).

## Publications

Check out our related [publications](docs/pages/publication.md) and how to cite them.

## They use manif

Find out [who's already using manif](docs/pages/projects.md).

## Contributing

Want to contribute? Great! Check out our [contribution guidelines](CONTRIBUTING.md).

[//]: # (URLs)

[jsola18]: http://arxiv.org/abs/1812.01537
[jsola18v]: http://arxiv.org/abs/1812.01537v4
[barrau15]: https://arxiv.org/pdf/1410.1465.pdf
[deray20]: https://joss.theoj.org/papers/10.21105/joss.01371

[jsola-iri-lecture]: https://www.youtube.com/watch?v=nHOcoIyJj2o
[jsola-iros-workshop]: https://www.youtube.com/watch?v=QR1p0Rabuww
[IRI-UPC]: https://www.iri.upc.edu/

[eigen]: http://eigen.tuxfamily.org
[ceres]: http://ceres-solver.org/
[ceres-jet]: http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets
[autodiff]: https://autodiff.github.io/
[cppad]: https://coin-or.github.io/CppAD/doc/cppad.htm

[crtp]: https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern

[manif-repo]: https://github.com/artivis/manif.git
[manif-issue]: https://github.com/artivis/manif/issues
[manif-doc]: https://artivis.github.io/manif/
[manif-doc-cpp]: https://artivis.github.io/manif/cpp/index.html
[manif-doc-python]: https://artivis.github.io/manif/python/index.html
[cheat_sheet]: paper/Lie_theory_cheat_sheet.pdf

[optional-repo]: https://github.com/TartanLlama/optional

[pybind11]: https://pybind11.readthedocs.io/en/stable/index.html

[git-workflow]: http://nvie.com/posts/a-successful-git-branching-model/

[badge-ci]: https://github.com/artivis/manif/workflows/build-and-test/badge.svg?branch=devel
[badge-ci-img]: https://github.com/artivis/manif/workflows/build-and-test/badge.svg?branch=devel
[badge-ci-win]: https://ci.appveyor.com/project/artivis/manif
[badge-ci-win-img]: https://ci.appveyor.com/api/projects/status/l0q7b0shhonvejrd?svg=true
[badge-doc-img]: https://codedocs.xyz/artivis/manif.svg
[badge-cov]: https://codecov.io/gh/artivis/manif
[badge-cov-img]: https://codecov.io/gh/artivis/manif/branch/devel/graph/badge.svg
[badge-license]: https://img.shields.io/github/license/mashape/apistatus.svg
[badge-joss]: http://joss.theoj.org/papers/e3fc778689407f0edd19df8c2089c160
[badge-joss-img]: http://joss.theoj.org/papers/e3fc778689407f0edd19df8c2089c160/status.svg

[latex1]: https://latex.codecogs.com/svg.latex?\mathbf&amp;space;\mathcal{X}^{-1}
[latex2]: https://latex.codecogs.com/svg.latex?\mathbf&amp;space;\mathcal{X}&amp;space;\circ&amp;space;\mathbf&amp;space;\mathcal{Y}
[latex3]: https://latex.codecogs.com/svg.latex?\varphi^\wedge
[latex4]: https://latex.codecogs.com/svg.latex?\mathbf\mathcal{X}\circ\mathbf&amp;space;v
[latex5]: https://latex.codecogs.com/svg.latex?\exp(\mathbf\varphi^\wedge)
[latex6]: https://latex.codecogs.com/svg.latex?\log(\mathbf&amp;space;\mathcal{X})^\vee
[latex7]: https://latex.codecogs.com/svg.latex?\operatorname{Adj}(\mathbf&amp;space;\mathcal{X})
[latex8]: https://latex.codecogs.com/svg.latex?\operatorname{adj}(\mathbf&amp;space;\varphi^\wedge)
[latex9]: https://latex.codecogs.com/svg.latex?\mathbf\mathcal{X}\oplus\mathbf\varphi=\mathbf\mathcal{X}\circ\exp(\mathbf\varphi^\wedge)
[latex10]: https://latex.codecogs.com/svg.latex?\mathbf\varphi\oplus\mathbf\mathcal{X}=\exp(\mathbf\varphi^\wedge)\circ\mathbf\mathcal{X}
[latex11]: https://latex.codecogs.com/svg.latex?\mathbf\mathcal{X}\ominus\mathbf\mathcal{Y}=\log(\mathbf\mathcal{Y}^{-1}\circ\mathbf\mathcal{X})^\vee
[latex12]: https://latex.codecogs.com/svg.latex?\mathbf\mathcal{X}\ominus\mathbf\mathcal{Y}=\log(\mathbf\mathcal{X}\circ\mathbf\mathcal{Y}^{-1})^\vee\phantom{.}
[latex13]: https://latex.codecogs.com/svg.latex?\mathbf\mathcal{X}^{-1}\circ\mathbf\mathcal{Y}
[latex14]: https://latex.codecogs.com/svg.latex?\langle\varphi,\tau\rangle
[latex15]: https://latex.codecogs.com/svg.latex?\left\lVert\varphi\right\rVert
[latex16]: https://latex.codecogs.com/svg.latex?\mathbf\mathcal{X},\mathbf\mathcal{Y}
[latex17]: https://latex.codecogs.com/svg.latex?\mathbf\varphi^\wedge,\tau^\wedge
[latex18]: https://latex.codecogs.com/svg.latex?\mathbf\varphi,\tau
[latex19]: https://latex.codecogs.com/svg.latex?\mathbb{R}^n
[latex20]: https://latex.codecogs.com/svg.latex?\mathbf{v}
[latex21]: https://latex.codecogs.com/svg.latex?\mathbb{R}^n
[latex22]: https://latex.codecogs.com/svg.latex?\frac{\delta&amp;space;f(\mathbf\mathcal{X})}{\delta\mathbf\mathcal{X}}\triangleq\lim_{\varphi\to0}\frac{&amp;space;f(\mathbf\mathcal{X}\oplus\varphi)\ominus&amp;space;f(\mathbf\mathcal{X})}{\varphi}\triangleq\lim_{\varphi\to0}\frac{\log(f(\mathbf\mathcal{X})^{-1}&amp;space;f(\mathbf\mathcal{X}\exp(\varphi^\wedge)))^\vee}{\varphi}
[latex23]: https://latex.codecogs.com/svg.latex?SO(3)
