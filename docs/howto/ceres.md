# Use with Ceres

The **manif** package differentiates Jacobians with respect to a
perturbation on the local tangent space.

```{important}
To understand why is this important,
especially when using manif with non-linear solvers,
make sure to read the ['autodiff'](../explanation/autodiff.md) explanation page.
```

## Jacobians

In [`Ceres`][ceres]' framework,
the computation of the Jacobian with respect to a
local perturbation on the tangent space is decoupled
in two folds as explained hereafter.

### Cost function

A Ceres [`CostFunction`][ceres-costfunction]
is a class implementing a function $f({\bf\mathcal{X}})$ such as,

```cpp
class QuadraticCostFunction : public ceres::SizedCostFunction<1, 1> {
 public:
  virtual ~QuadraticCostFunction() {}
  virtual bool Evaluate(
    double const* const* parameters, double* residuals, double** jacobians
  ) const {
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

It produces the intermediate Jacobian ['(2)' detailed in 'autodiff'](../explanation/autodiff.md).

### Local parameterization

In Ceres, a [`LocalParameterization`][ceres-localparam] can be associated to a state.
For instance:

```cpp
Eigen::Quaterniond my_state;

ceres::Problem::Options problem_options;
ceres::Problem problem(problem_options);

// Add the state to Ceres problem
problem->AddParameterBlock(my_state.data(), 4);

// Associate a LocalParameterization to the state vector
problem_->SetManifold(
  my_state.data(), new EigenQuaternionParameterization()
);
```

The `LocalParameterization` class (and derived) performs the state update step
of the optimization. If also computes the associated Jacobian which is evaluated at ${\boldsymbol\omega}={\bf 0}$.

Once both the `CostFunction` and `LocalParameterization`'s Jacobians are evaluated,
`Ceres` internally computes the Jacobian (with respect to a perturbation on the local tangent space) as the product ['(4)' detailed in 'autodiff'](../explanation/autodiff.md).

Voila.

The intermediate Jacobians that `Ceres` requires are **not** available in **manif**
since it provides directly the final Jacobian.

However, one still wants to use **manif** with in a `Ceres`-based project.
To that end, **manif** is compliant with `Ceres`
auto-differentiation and the [`ceres::Jet`][ceres-jet] type.

Below are presented two small examples illustrating how **manif** can be used with `Ceres`.

## A group-abstract LocalParameterization

In the snippet below is shown how one can implement a template
`ceres::LocalParameterization`-derived class,
thus working for any group.

```cpp
template <typename _LieGroup>
class CeresLocalParameterization {
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
  bool operator()(
    const T* state_raw, const T* delta_raw, T* state_plus_delta_raw
  ) const {
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

## A small Ceres problem

The example below highlights the use of the predefined `Ceres`
helper classes available in **manif**.
In this example,
we compute an average from 4 points in `SE2`.

```cpp
// Tell ceres not to take ownership of the raw pointers
ceres::Problem::Options problem_options;
problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
problem_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

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
problem.AddResidualBlock(
  obj_pi_over_4.get(), nullptr, average_state.data()
);

problem.AddResidualBlock(
  obj_3_pi_over_8.get(), nullptr, average_state.data()
);

problem.AddResidualBlock(
  obj_5_pi_over_8.get(), nullptr,  average_state.data()
);

problem.AddResidualBlock(
  obj_3_pi_over_4.get(), nullptr, average_state.data()
);

// We use a second manif helper that creates a ceres local parameterization
// for our optimized state block.

std::shared_ptr<ceres::Manifold> auto_diff_manifold = manif::make_manifold_autodiff<SE2d>();

problem.SetManifold(average_state.data(), auto_diff_manifold.get());

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
