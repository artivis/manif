# Ceres

While the `manif` package differentiates Jacobians with respect to a
local perturbation on the tangent space,
many non-linear solvers
(including [Ceres][ceres]) expect them to be differentiated
with respect to the underlying representation vector of the group element
(e.g. wrt the quaternion parameters vector
for ![SO3][latex1].

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

In Ceres' framework, the computation of this Jacobian is decoupled
in two folds as explained hereafter.
The following terminology should sounds familiar to Ceres users.

Ceres [`CostFunction`][ceres-costfunction]
is the class representing and implementing a function ![f(x)][latex9],
for instance,

```cpp
class QuadraticCostFunction : public ceres::SizedCostFunction<1, 1> {
 public:
  virtual ~QuadraticCostFunction() {}
  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    const double x = parameters[0][0];
    residuals[0] = 10 - x;

    // Compute the Jacobian if asked for.
    if (jacobians != NULL && jacobians[0] != NULL) {
      jacobians[0][0] = -1;
    }
    return true;
  }
};
```

It produces the Jacobian,

![J_e_x(+)omega][latex10]

In Ceres, a [`LocalParameterization`][ceres-localparam] can be associated to a state.

```cpp
Eigen::Quaterniond my_state;

ceres::Problem::Options problem_options;
ceres::Problem problem(problem_options);

// Add the state to Ceres problem
problem->AddParameterBlock(my_state.data(), 4);

// Associate a LocalParameterization to the state vector
problem_->SetParameterization(my_state.data(),
                              new EigenQuaternionParameterization() );
```

The `LocalParameterization` class (and derived) performs the state update step
of the optimization - the ![x(+)omega][latex11] operation.
While the function operates for any ![omega][latex12],
its Jacobian is evaluated for ![omega=0][latex13] thus providing the Jacobian,

![J_x(+)w_w][latex14]

Once both the `CostFunction` and `LocalParameterization`'s Jacobians are evaluated,
`Ceres` internally computes `(1)` with the following product,

![J_e_w = J_e_x(+)omega * J_x(+)w_w][latex15]

Voila.

The intermediate Jacobians (2-3) that `Ceres` requires are **not** available in `manif`
since it provide directly the final Jacobian detailed in `(1)`.

However, one still wants to use `manif` with his `Ceres`-based project.
For this reason, `manif` is compliant with `Ceres`
auto-differentiation and the [`ceres::Jet`][ceres-jet] type to compute (2-3).

Below are presented two small examples illustrating how `manif` can be used with `Ceres`.

## Example : A group-abstract `LocalParameterization`

Is shown here how one can implement a
`ceres::LocalParameterization`-derived class using `manif`,
that does the ![x(+)omega][latex16] for any group implemented in `manif` passed as a template parameter.

```cpp
template <typename _LieGroup>
class CeresLocalParameterization
{
  using LieGroup = _LieGroup;
  using Tangent  = typename _LieGroup::Tangent;

  template <typename _Scalar>
  using LieGroupTemplate = typename LieGroup::template LieGroupTemplate<_Scalar>;

  template <typename _Scalar>
  using TangentTemplate = typename Tangent::template TangentTemplate<_Scalar>;

public:

  CeresLocalParameterizationFunctor() = default;
  virtual ~CeresLocalParameterizationFunctor() = default;

  template<typename T>
  bool operator()(const T* state_raw,
                  const T* delta_raw,
                  T* state_plus_delta_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> state(state_raw);
    const Eigen::Map<const TangentTemplate<T>>  delta(delta_raw);

    Eigen::Map<LieGroupTemplate<T>> state_plus_delta(state_plus_delta_raw);

    state_plus_delta = state + delta;

    return true;
  }
};
//
...
// Some typedef helpers
using CeresLocalParameterizationSO2 = CeresLocalParameterizationFunctor<SO2d>;
using CeresLocalParameterizationSE2 = CeresLocalParameterizationFunctor<SE2d>;
using CeresLocalParameterizationSO3 = CeresLocalParameterizationFunctor<SO3d>;
using CeresLocalParameterizationSE3 = CeresLocalParameterizationFunctor<SE3d>;
```

## Example : A small Ceres problem

This example highlights the use of the predefined `Ceres`
helper classes available with `manif`.
In this example, we compute an average point from 4 points in `SE2`.

```cpp
// Tell ceres not to take ownership of the raw pointers
ceres::Problem::Options problem_options;
problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

ceres::Problem problem(problem_options);

// We use a first manif helper that creates a ceres cost-function.
// The cost function computes the distance between
// the desired state and the current state

// Create 4 objectives which are 'close' in SE2.
std::shared_ptr<ceres::CostFunction> obj_pi_over_4   = manif::make_objective_autodiff<SE2d>(3, 3,    M_PI/4.);
std::shared_ptr<ceres::CostFunction> obj_3_pi_over_8 = manif::make_objective_autodiff<SE2d>(3, 1, 3.*M_PI/8.);
std::shared_ptr<ceres::CostFunction> obj_5_pi_over_8 = manif::make_objective_autodiff<SE2d>(1, 1, 5.*M_PI/8.);
std::shared_ptr<ceres::CostFunction> obj_3_pi_over_4 = manif::make_objective_autodiff<SE2d>(1, 3, 3.*M_PI/4.);

SE2d average_state(0,0,0);

/////////////////////////////////

// Add residual blocks to ceres problem
problem.AddResidualBlock( obj_pi_over_4.get(),
                          nullptr,
                          average_state.data() );

problem.AddResidualBlock( obj_3_pi_over_8.get(),
                          nullptr,
                          average_state.data() );

problem.AddResidualBlock( obj_5_pi_over_8.get(),
                          nullptr,
                           average_state.data() );

problem.AddResidualBlock( obj_3_pi_over_4.get(),
                          nullptr,
                          average_state.data() );

// We use a second manif helper that creates a ceres local parameterization
// for our optimized state block.

std::shared_ptr<ceres::LocalParameterization>
  auto_diff_local_parameterization =
    manif::make_local_parametrization_autodiff<SE2d>();

problem.SetParameterization( average_state.data(),
                             auto_diff_local_parameterization.get() );

// Run the solver!
ceres::Solver::Options options;
options.minimizer_progress_to_stdout = true;

ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);

std::cout << "summary:\n" << summary.FullReport() << "\n";

std::cout << "Average state:\nx:" << average_state.x()
          << "\ny:" << average_state.y()
          << "\nt:" << average_state.angle()
          << "\n\n";
```

[//]: # (URLs)

[ceres]: http://ceres-solver.org/
[ceres-costfunction]: http://ceres-solver.org/nnls_modeling.html#costfunction
[ceres-localparam]: http://ceres-solver.org/nnls_modeling.html#localparameterization
[ceres-jet]: http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets

[latex1]: https://latex.codecogs.com/png.latex?SO^3
[latex2]: https://latex.codecogs.com/png.latex?\bf&amp;space;x
[latex3]: https://latex.codecogs.com/png.latex?\omega
[latex4]: https://latex.codecogs.com/png.latex?\bf&amp;space;x
[latex5]: https://latex.codecogs.com/png.latex?{\bf&amp;space;e}=f({\bf&amp;space;x})
[latex6]: https://latex.codecogs.com/png.latex?f({\bf&amp;space;x\oplus\omega})\approx{\bf&amp;space;e}+{\bf&amp;space;J}_{\omega}^{e}~\omega&amp;space;.
[latex7]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;J}_{\omega}^{e}=\frac{\delta{\bf&amp;space;e}}{\delta{\bf&amp;space;x}}=\frac{\delta&amp;space;f({\bf&amp;space;x})}{\delta{\bf&amp;space;x}}=\lim_{\omega\to0}\frac{f({\bf&amp;space;x}\oplus\omega)\ominus&amp;space;f({\bf&amp;space;x})}{\omega},&amp;space;(1)
[latex8]: https://latex.codecogs.com/png.latex?f({\bf&amp;space;x})
[latex9]: https://latex.codecogs.com/png.latex?{\bf&amp;space;e}=f({\bf&amp;space;x})
[latex10]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;J}_{{\bf&amp;space;x}\oplus\omega}^{e}=\frac{\delta{\bf&amp;space;e}}{\delta({\bf&amp;space;x}\oplus\omega)}=\lim_{\mathbf&amp;space;h\to0}\frac{&amp;space;f({\bf&amp;space;x}+\mathbf&amp;space;h)-f({\bf&amp;space;x})}{\mathbf&amp;space;h}.&amp;space;(2)
[latex11]: https://latex.codecogs.com/png.latex?{\bf&amp;space;x}\oplus\mathbf\omega
[latex12]: https://latex.codecogs.com/png.latex?\mathbf\omega
[latex13]: https://latex.codecogs.com/png.latex?\omega=0
[latex14]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;J}_{\omega}^{{\bf&amp;space;x}\oplus\omega}=\frac{\delta({\bf&amp;space;x}\oplus\omega)}{\delta\omega}=\lim_{\delta\omega\to0}\frac{{\bf&amp;space;x}\oplus(\omega+\delta\omega)-{\bf&amp;space;x}\oplus\mathbf\omega}{\delta\omega}=\lim_{\delta\omega\to0}\frac{{\bf&amp;space;x}\oplus\delta\omega-{\bf&amp;space;x}}{\delta\omega}.&amp;space;(3)
[latex15]: https://latex.codecogs.com/svg.latex?{\bf&amp;space;J}_{\omega}^{e}={\bf&amp;space;J}_{{\bf&amp;space;x}\oplus\omega}^{e}\times{\bf&amp;space;J}_{\omega}^{{\bf&amp;space;x}\oplus\omega}.&amp;space;(4)
[latex16]: https://latex.codecogs.com/png.latex?{\bf&amp;space;x}\oplus\mathbf\omega
