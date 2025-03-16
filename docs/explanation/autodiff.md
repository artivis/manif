# Notes on auto-differentiation

The **manif** package differentiates Jacobians with respect to a
**local perturbation on the tangent space**.
These Jacobians, map tangent spaces as described in [the paper][jsola18].

However, many non-linear solvers
(e.g. [Ceres][ceres]) expect functions to be differentiated with respect to the underlying
representation vector of the group element
(e.g. with respect to quaternion vector for `SO3`).

For this reason,
**manif** is compliant with the auto-differentiation libraries
[`ceres::Jet`][ceres-jet], [`autodiff::Dual`][autodiff] & [`autodiff::Real`][autodiff].

For reference,
**manif** implements rotations in the following way:

- SO(2) and SE(2): as a complex number with `real = cos(theta)` and `imag = sin(theta)` values.
- SO(3), SE(3) and SE_2(3): as a unit quaternion, using the underlying `Eigen::Quaternion` type.

Therefore, the respective Jacobian sizes using [`autodiff::dual`][autodiff] are as follows:

| Group | Size (SxS) |
| :---    | :---: |
| ℝ(n)    |  n |
| SO(2)   |  2 |
| SO(3)   |  4 |
| SE(2)   |  4 |
| SE(3)   |  7 |
| SE_2(3) | 10 |
| SGal(3) | 11 |

## Jacobians

Considering, {math}`\bf x` a group element (e.g. S3),
{math}`\omega` the vector tangent to the group at {math}`\bf x`,
{math}`f({\bf x})` an error function,
one is interested in expressing the Taylor series of the error function,
{math}`f({\bf x}\oplus\omega)`.

Therefore we have to compute

```{math}
{\bf J}_{\omega}^{e}=\frac{\delta{\bf e}}{\delta{\bf x}}=\frac{\delta f({\bf x})}{\delta{\bf x}}=\lim_{\omega\to0}\frac{f({\bf x}\oplus\omega)\ominus f({\bf x})}{\omega}, (1)
```

the **Jacobian of** {math}`f({\bf x})` **with respect to a perturbation on the tangent space**,
so that the state update happens on the manifold tangent space.

In some optimization frameworks,
the computation of this Jacobian is decoupled in two folds as explained hereafter.

Using the [autodiff][autodiff] library,
a cost function can straightforwardly be designed as follows:

```cpp
// functor to be evaluated
auto fun = [](const auto& measurement, const auto& state_i, const auto& state_j){
  return measurement - (state_j - state_i);
};
```

where `state_i` & `state_j` belong to a group
and `measurement` belongs to the group's tangent.

Evaluating the function and its Jacobians is,

```cpp
using namespace autodiff;
Eigen::MatrixXd J_e_xi = jacobian(fun, wrt(xi), at(meas_ij, xi, xj), e);
Eigen::MatrixXd J_e_xj = jacobian(fun, wrt(xj), at(meas_ij, xi, xj), e);
```

It produces Jacobians of the form,

```{math}
{\bf J}_{{\bf x}\oplus\omega}^{e}=\frac{\delta{\bf e}}{\delta({\bf x}\oplus\omega)}=\lim_{\mathbf h\to0}\frac{ f({\bf x}+\mathbf h)-f({\bf x})}{\mathbf h}, (2)
```

We thus then need to compute the Jacobian that will map to the tangent space -
often called local-parameterization.
A convenience function is provided in **manif** to do so as follow:

```cpp
Eigen::MatrixXd J_xi_lp = autodiffLocalParameterizationJacobian<dual>(xi);
Eigen::MatrixXd J_xj_lp = autodiffLocalParameterizationJacobian<dual>(xj);
```

This function computes the {math}`{\bf x}\oplus\mathbf\omega` operation's
Jacobian evaluated for {math}`\omega=0` thus providing the Jacobian,

```{math}
{\bf J}_{\omega}^{{\bf x}\oplus\omega}=\frac{\delta({\bf x}\oplus\omega)}{\delta\omega}=\lim_{\delta\omega\to0}\frac{{\bf x}\oplus(\omega+\delta\omega)-{\bf x}\oplus\mathbf\omega}{\delta\omega}=\lim_{\delta\omega\to0}\frac{{\bf x}\oplus\delta\omega-{\bf x}}{\delta\omega}, (3)
```

Once both the cost function and local-parameterization's Jacobians are evaluated,
they can be compose as,

```{math}
{\bf J}_{\omega}^{e}={\bf J}_{{\bf x}\oplus\omega}^{e}\times{\bf J}_{\omega}^{{\bf x}\oplus\omega}, (4)
```

Voila.

The intermediate Jacobians `(2-3)` that some solver requires are **not** available in **manif**
since the library provides directly the final Jacobian `(1)`.

However, **manif** is compliant with the auto-differentiation libraries
[`ceres::Jet`][ceres-jet], [`autodiff::Dual`][autodiff] & [`autodiff::Real`][autodiff] to compute `(2-3)`.

[//]: # (URLs)

[jsola18]: http://arxiv.org/abs/1812.01537

[ceres]: http://ceres-solver.org/
[autodiff]: https://autodiff.github.io/
[ceres-jet]: http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets
