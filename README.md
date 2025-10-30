# manif

## A small header-only library for Lie theory

[![GHA][badge-ci-img]][badge-ci]
[![appveyor][badge-ci-win-img]][badge-ci-win]
[![Documentation][badge-doc-img]][manif-doc]
[![codecov][badge-cov-img]][badge-cov]
![GitHub][badge-license]
[![JOSS][badge-joss-img]][deray20]

## Package Summary

<!-- Include start manif intro -->

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
- SGal(3): The Special Galilean group (rotation, translation, velocity and time) in 3D space,
  described in these papers [[1][fourmy19]] & [[2][kelly24]].
- Bundle<>: allows manipulating a manifold bundle as a single Lie group.
  Referred to as a *composite manifold* in Section IV of the
  [reference paper](http://arxiv.org/abs/1812.01537).

Other Lie groups can and will be added, contributions are welcome.

**manif** is based on the mathematical presentation of the Lie theory available in [this paper][jsola18].
We recommend every user of **manif** to read the paper (17 pages) before starting to use the library.
The paper offers a comprehensive yet accessible introduction to Lie theory, tailored specifically for roboticists.
By presenting the material in a simplified manner,
it lowers the barrier to entry for those interested in developing rigorous and elegant algorithms for state estimation,
control, learning, and possibly more.
This approach ensures that even readers without an advanced mathematical background can grasp the
foundational concepts and apply them effectively in robotics.

<!-- Include stop manif intro -->

You may also find the following video online,
['Lie theory for the roboticist'][jsola-iri-lecture],
a lecture given at [IRI-UPC][IRI-UPC].

> In a rush? Check out our [Lie group cheat sheet][cheat_sheet].

**manif** provides analytic computation of Jacobians for all the operations listed [below](#features).

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

<!-- Include start manif operation -->

| Operation  |       | Code |
| :---       |   :---:   | :---: |
|       |   Base Operation   |  |
| Inverse | $\bf\mathcal{X}^{-1}$ | `X.inverse()` |
| Composition | $\bf\mathcal{X}\circ\bf\mathcal{Y}$ | `X * Y`<br/>`X.compose(Y)` |
| Hat | $\boldsymbol\varphi^\wedge$ | `w.hat()` |
| Act on vector | $\bf\mathcal{X}\circ{\bf v}$ | `X.act(v)` |
| Retract to group element | $\exp(\boldsymbol\varphi^\wedge)$ | `w.exp()` |
| Lift to tangent space | $\log(\bf\mathcal{X})^\vee$ | `X.log()` |
| Manifold Adjoint | $\mathrm{Adj}(\bf\mathcal{X})$ | `X.adj()` |
| Tangent adjoint | $\mathrm{adj}(\boldsymbol\varphi^\wedge)$ | `w.smallAdj()` |
|       |   Composed Operation   |  |
| Manifold right plus | ${\bf\mathcal{X}}\circ\exp(\boldsymbol\varphi^\wedge)$ | `X + w`<br/>`X.plus(w)`<br/>`X.rplus(w)` |
| Manifold left plus | $\exp(\boldsymbol\varphi^\wedge)\circ\bf\mathcal{X}$ | `w + X`<br/>`w.plus(X)`<br/>`w.lplus(X)` |
| Manifold right minus | $\log(\bf\mathcal{Y}^{-1}\circ\bf\mathcal{X})^\vee$ | `X - Y`<br/>`X.minus(Y)`<br/>`X.rminus(Y)` |
| Manifold left minus | $\log(\bf\mathcal{X}\circ\bf\mathcal{Y}^{-1})^\vee$ | `X.lminus(Y)` |
| Between | ${\bf\mathcal{X}^{-1}}\circ{\bf\mathcal{Y}}$ | `X.between(Y)` |
| Inner Product | $\langle\boldsymbol\varphi,\boldsymbol\tau\rangle$ | `w.inner(t)` |
| Norm | $\left\lVert\boldsymbol\varphi\right\rVert$ | `w.weightedNorm()`<br/>`w.squaredWeightedNorm()` |

Above, ${\bf\mathcal{X}}$ & ${\bf\mathcal{Y}}$ (`X` & `Y`) represent group elements,
${\boldsymbol\varphi^\wedge}$ & ${\boldsymbol\tau^\wedge}$ represent elements in the Lie algebra of the Lie group,
${\boldsymbol\varphi}$ & ${\boldsymbol\tau}$ (`w` & `t`) represent the same elements of the tangent space
but expressed in Cartesian coordinates in $\mathbb{R}^n$,
and $\mathbf{v}$ (`v`) represents any element of $\mathbb{R}^n$.

<!-- Include stop manif operation -->

### Tangent spaces

<!-- Include start manif tangent -->

**manif** favors Cartesian representations of the tangent spaces.
This means that the tangent elements are regular vectors in $\mathbb{R}^n$,
'n' being the dimension of the Lie group.

The ordering of the elements in such vectors matters to correctly interpret them.
It impacts the form of all Jacobian matrices and covariances matrices that will be defined on those tangent spaces.

As a reference, this is the way tangent spaces are defined in **manif**

| group | dimension | group elements | tangent elements (in order) | 
| ---- | ---- | ---- | ---- | 
| Rn | n | $\bf p$ | $\bf p$ | 
| SO(2) | 1 | $\bf R$ | $\theta$ | 
| SO(3) | 3 | $\bf R$ | $\boldsymbol\theta$ | 
| SE(2) | 3 | $\bf p$, $\bf R$ | $\boldsymbol\rho$, $\theta$ | 
| SE(3) | 6 | $\bf p$, $\bf R$ | $\boldsymbol\rho$, $\boldsymbol\theta$ | 
| SE_2(3) | 9 | $\bf p$, $\bf R$, $\bf v$ | $\boldsymbol\rho$, $\boldsymbol\theta$, $\boldsymbol\nu$ | $
| SGal(3) | 10 | $\bf p$, $\bf R$, $\bf v$, $t$ | $\boldsymbol\rho$, $\boldsymbol\nu$, $\boldsymbol\theta$, $s$ | 

NOTE: the unfortunate order mismatch between the tangent elements in SE_2(3) and SGal(3).

As an example, in SE_2(3) the tangent vector ${\boldsymbol\tau}$ is defined by

$$
{\boldsymbol\tau} =
\begin{bmatrix}
{\boldsymbol\rho} \\
{\boldsymbol\theta} \\
{\boldsymbol\nu}
\end{bmatrix} \in \mathbb{R}^9
$$

where $\boldsymbol\rho$, $\boldsymbol\theta$ and $\boldsymbol\nu$ are $\in \mathbb{R}^3$ and
typically correspond respectively to changes in position, orientation and velocity.

A covariances matrix $\bf Q$ of an element of SE_2(3) can be block-partitioned as follows

$$
{\bf Q} = \begin{bmatrix}
  {\bf Q}_ {\boldsymbol\rho\boldsymbol\rho} & {\bf Q}_ {\boldsymbol\rho\boldsymbol\theta} & {\bf Q}_ {\boldsymbol\rho\boldsymbol\nu} \\
  {\bf Q}_ {\boldsymbol\theta\boldsymbol\rho} & {\bf Q}_ {\boldsymbol\theta\boldsymbol\theta} & {\bf Q}_ {\boldsymbol\theta\boldsymbol\nu} \\
  {\bf Q}_ {\boldsymbol\nu\boldsymbol\rho} & {\bf Q}_ {\boldsymbol\nu\boldsymbol\theta} & {\bf Q}_ {\boldsymbol\nu\boldsymbol\nu}
  \end{bmatrix} \in \mathbb{R}^{9\times 9}
$$

All blocks ${\bf Q}_{\bf ij}$ are $3\times3$ and ${\bf Q}$ is $9\times9$.

<!-- Include stop manif tangent -->

### Jacobians

All operations come with their respective analytical Jacobian matrices.
Throughout **manif**, **Jacobians are differentiated with respect to a perturbation on the local tangent space**.
These Jacobians map tangent spaces, as described in [this paper][jsola18].
Please consider [the order of elements in the tangent spaces](#tangent-spaces) when manipulating Jacobians.

Currently, **manif** implements the **right Jacobian**, whose definition reads:

$$
\frac{\delta f(\bf\mathcal{X})}{\delta\bf\mathcal{X}}\triangleq
\lim_{\boldsymbol\varphi\to\bf0}\frac{f(\bf\mathcal{X}\oplus\boldsymbol\varphi)\ominus f(\bf\mathcal{X})}{\boldsymbol\varphi}\triangleq
\lim_{\boldsymbol\varphi\to\bf0}\frac{\log(f({\bf\mathcal{X}})^{-1} f({\bf\mathcal{X}}\exp(\boldsymbol\varphi^\wedge)))^\vee}{\boldsymbol\varphi}
$$

The Jacobians of any of the aforementioned operations can then be evaluated:

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
(e.g. [Ceres][ceres]) expect functions to be differentiated with respect to
the underlying representation vector of the group element
(e.g. with respect to quaternion vector for `SO3`).

For this reason, **manif** is compliant with the auto-differentiation libraries
[`ceres::Jet`][ceres-jet], [`autodiff::Dual`][autodiff] & [`autodiff::Real`][autodiff].

## Documentation

The documentation is available online at the accompanying [website][manif-doc].
Both the [C++][manif-doc-cpp] and the [Python][manif-doc-python] APIs are documented.

Do you want to build it locally?
Find out how on the [dedicated page](docs/pages/documentation.md).

Note: throughout the code documentation we refer to 'the paper' which you can
find on [the dedicated page](docs/pages/publication.md).

## Tutorials and application demos

We provide some self-contained and self-explained
[C++ examples](docs/pages/cpp/Quick-start.md#tutorials-and-application-demos) to help you get started.

You prefer Python? The same examples are also
[available in Python](docs/pages/python/Quick-start.md#tutorials-and-application-demos).

## Publications

Check out our related [publications](docs/pages/publication.md) and how to cite them.

## They use manif

Find out [who's already using manif](docs/pages/projects.md).

## Contributing

Want to contribute? Great! Check out our [contribution guidelines](CONTRIBUTING.md).

[//]: # (URLs)

[jsola18]: http://arxiv.org/abs/1812.01537
[barrau15]: https://arxiv.org/pdf/1410.1465.pdf
[fourmy19]: https://hal.science/hal-02183498/document
[kelly24]: https://arxiv.org/abs/2312.07555
[deray20]: https://joss.theoj.org/papers/10.21105/joss.01371

[jsola-iri-lecture]: https://www.youtube.com/watch?v=nHOcoIyJj2o
[IRI-UPC]: https://www.iri.upc.edu/

[ceres]: http://ceres-solver.org/
[ceres-jet]: http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets
[autodiff]: https://autodiff.github.io/

[manif-repo]: https://github.com/artivis/manif.git
[manif-issue]: https://github.com/artivis/manif/issues
[manif-doc]: https://artivis.github.io/manif/
[manif-doc-cpp]: https://artivis.github.io/manif/cpp/index.html
[manif-doc-python]: https://artivis.github.io/manif/python/index.html
[cheat_sheet]: paper/Lie_theory_cheat_sheet.pdf

[badge-ci]: https://github.com/artivis/manif/workflows/build-and-test/badge.svg?branch=devel
[badge-ci-img]: https://github.com/artivis/manif/workflows/build-and-test/badge.svg?branch=devel
[badge-ci-win]: https://ci.appveyor.com/project/artivis/manif
[badge-ci-win-img]: https://ci.appveyor.com/api/projects/status/l0q7b0shhonvejrd?svg=true
[badge-doc-img]: https://codedocs.xyz/artivis/manif.svg
[badge-cov]: https://codecov.io/gh/artivis/manif
[badge-cov-img]: https://codecov.io/gh/artivis/manif/branch/devel/graph/badge.svg
[badge-license]: https://img.shields.io/github/license/mashape/apistatus.svg
[badge-joss-img]: https://joss.theoj.org/papers/10.21105/joss.01371/status.svg
