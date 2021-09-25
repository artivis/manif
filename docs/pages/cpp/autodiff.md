# Autodiff

- [Autodiff](#autodiff)
  - [Jacobians](#jacobians)

The **manif** package differentiates Jacobians with respect to a
local perturbation on the tangent space.
These Jacobians map tangent spaces, as described in [this paper][jsola18].

However, many non-linear solvers
(e.g. [Ceres][ceres]) expect functions to be differentiated with respect to the underlying
representation vector of the group element
(e.g. with respect to quaternion vector for `SO3`).

For this reason **manif** is compliant with the [`autodiff::dual`][autodiff]
auto-differentiation type.

For reference of the size of the Jacobians returned when using [`autodiff::dual`][autodiff],
**manif** implements rotations in the following way:

- SO(2) and SE(2): as a complex number with `real = cos(theta)` and `imag = sin(theta)` values.
- SO(3), SE(3) and SE_2(3): as a unit quaternion, using the underlying `Eigen::Quaternion` type.

Therefore, the respective Jacobian sizes using [`autodiff::dual`][autodiff] are as follows:

- ℝ(n) : size n
- SO(2) : size 2
- SO(3) : size 4
- SE(2) : size 4
- SE(3) : size 7
- SE_2(3): size 10

## Jacobians

Considering,

![x][latex2] a group element (e.g. S3),
![omega][latex3] the vector tangent to the group at ![x][latex4],
![f(x)][latex5] an error function,

one is interested in expressing the Taylor series of the error function,

![f(x(+)omega)][latex6]

Therefore we have to compute the Jacobian

![J_e_omega][latex7]

the **Jacobian of** ![f(x)][latex8] **with respect to a perturbation on the tangent space**,
so that the state update happens on the manifold tangent space.

In some optimization frameworks, the computation of this Jacobian is decoupled
in two folds as explained hereafter.

Using the **autodiff** library,
a cost function can very easily designed as follows,

```cpp
// functor to be evaluated
auto fun = [](const auto& measurement, const auto& state_i, const auto& state_j){
  return measurement - (state_j - state_i);
};
```

where `state_i` & `state_j` belong to a group
and `measurement` belongs to the group's tangent.

Evaluating the function and its Jacobians is then,

```cpp
using namespace autodiff;
Eigen::MatrixXd J_e_xi = jacobian(fun, wrt(xi), at(meas_ij, xi, xj), e);
Eigen::MatrixXd J_e_xj = jacobian(fun, wrt(xj), at(meas_ij, xi, xj), e);
```

It produces Jacobians of the form,

![J_e_x(+)omega][latex10]

We thus then need to compute the Jacobian that will map to the tangent space -
often called local-parameterization.
A convenience function is provided in **manif** to do so as follow:

```cpp
Eigen::MatrixXd J_xi_lp = autodiffLocalParameterizationJacobian<dual>(xi);
Eigen::MatrixXd J_xj_lp = autodiffLocalParameterizationJacobian<dual>(xj);
```

This function computes the ![x(+)omega][latex11] operation's
Jacobian evaluated for ![omega=0][latex13] thus providing the Jacobian,

![J_x(+)w_w][latex14]

Once both the cost function and local-parameterization's Jacobians are evaluated,
they can be compose as,

![J_e_w = J_e_x(+)omega * J_x(+)w_w][latex15]

Voila.

The intermediate Jacobians (2-3) that some solver requires are **not** available in `manif`
since the library provides directly the final Jacobian `(1)`.

However, **manif** is compliant with `autodiff::dual`
auto-differentiation type to compute (2-3).

[//]: # (URLs)

[jsola18]: http://arxiv.org/abs/1812.01537

[ceres]: http://ceres-solver.org/
[ceres-costfunction]: http://ceres-solver.org/nnls_modeling.html#costfunction
[ceres-localparam]: http://ceres-solver.org/nnls_modeling.html#localparameterization
[ceres-jet]: http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets
[autodiff]: https://autodiff.github.io/

[latex1]: https://latex.codecogs.com/svg.latex?SO^3
[latex2]: https://latex.codecogs.com/svg.latex?\bf&amp;space;x
[latex3]: https://latex.codecogs.com/svg.latex?\omega
[latex4]: https://latex.codecogs.com/svg.latex?\bf&amp;space;x
[latex5]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;e}=f({\bf&amp;space;x})
[latex6]: https://latex.codecogs.com/svg.latex?f({\bf&amp;space;x\oplus\omega})\approx{\bf&amp;space;e}+{\bf&amp;space;J}_{\omega}^{e}~\omega&amp;space;.
[latex7]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;J}_{\omega}^{e}=\frac{\delta{\bf&amp;space;e}}{\delta{\bf&amp;space;x}}=\frac{\delta&amp;space;f({\bf&amp;space;x})}{\delta{\bf&amp;space;x}}=\lim_{\omega\to0}\frac{f({\bf&amp;space;x}\oplus\omega)\ominus&amp;space;f({\bf&amp;space;x})}{\omega},&amp;space;(1)
[latex8]: https://latex.codecogs.com/svg.latex?f({\bf&amp;space;x})
[latex9]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;e}=f({\bf&amp;space;x})
[latex10]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;J}_{{\bf&amp;space;x}\oplus\omega}^{e}=\frac{\delta{\bf&amp;space;e}}{\delta({\bf&amp;space;x}\oplus\omega)}=\lim_{\mathbf&amp;space;h\to0}\frac{&amp;space;f({\bf&amp;space;x}+\mathbf&amp;space;h)-f({\bf&amp;space;x})}{\mathbf&amp;space;h}.&amp;space;(2)
[latex11]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;x}\oplus\mathbf\omega
[latex12]: https://latex.codecogs.com/svg.latex?\mathbf\omega
[latex13]: https://latex.codecogs.com/svg.latex?\omega=0
[latex14]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;J}_{\omega}^{{\bf&amp;space;x}\oplus\omega}=\frac{\delta({\bf&amp;space;x}\oplus\omega)}{\delta\omega}=\lim_{\delta\omega\to0}\frac{{\bf&amp;space;x}\oplus(\omega+\delta\omega)-{\bf&amp;space;x}\oplus\mathbf\omega}{\delta\omega}=\lim_{\delta\omega\to0}\frac{{\bf&amp;space;x}\oplus\delta\omega-{\bf&amp;space;x}}{\delta\omega}.&amp;space;(3)
[latex15]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;J}_{\omega}^{e}={\bf&amp;space;J}_{{\bf&amp;space;x}\oplus\omega}^{e}\times{\bf&amp;space;J}_{\omega}^{{\bf&amp;space;x}\oplus\omega}.&amp;space;(4)
[latex16]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;x}\oplus\mathbf\omega
