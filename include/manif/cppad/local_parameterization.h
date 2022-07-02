#ifndef _MANIF_MANIF_CPPAD_LOCAL_PARAMETRIZATION_H_
#define _MANIF_MANIF_CPPAD_LOCAL_PARAMETRIZATION_H_

namespace manif {

enum class AutoDifferentiation : char {Aut, For, Rev};

/**
   * @brief Compute the local parameterization Jacobian
   * using CppAD auto differentiation.
   *
   * @tparam Derived
   * @param _state
   * @param m
   * @return Jacobian
   */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RepSize, Derived::DoF>
cppadLocalParameterizationJacobian(
  const manif::LieGroupBase<Derived>& _state,
  const AutoDifferentiation m = AutoDifferentiation::Aut
) {
  using Scalar = typename Derived::Scalar;
  using Ad = CppAD::AD<Scalar>;

  using LieGroup = typename Derived::template LieGroupTemplate<Ad>;
  using Tangent = typename Derived::Tangent::template TangentTemplate<Ad>;

  using Jacobian = Eigen::Matrix<
    Scalar,
    Derived::RepSize,
    Derived::DoF,
    (Derived::DoF>1)? Eigen::RowMajor : Eigen::ColMajor
  >;

  using MatrixXad = Eigen::Matrix<Ad, Eigen::Dynamic, 1>;

  const LieGroup state = _state.coeffs().template cast<Ad>();
  MatrixXad delta = MatrixXad::Zero(Derived::DoF);

  MatrixXad state_plus_delta(Derived::RepSize);

  CppAD::Independent(delta);

  Eigen::Map<LieGroup>(state_plus_delta.data()) =
    state + Eigen::Map<const Tangent>(delta.data());

  CppAD::ADFun<Scalar> ad_fun(delta, state_plus_delta);

  MANIF_ASSERT(state.coeffs().isApprox(state_plus_delta));

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> jac;

  switch (m) {
  case AutoDifferentiation::Aut:
    jac = ad_fun.Jacobian(delta.template cast<Scalar>().eval());
    break;
  case AutoDifferentiation::For:
    jac.resize(ad_fun.Domain()*ad_fun.Range());
    CppAD::JacobianFor(ad_fun, delta.template cast<Scalar>().eval(), jac);
    break;
  case AutoDifferentiation::Rev:
    jac.resize(ad_fun.Domain()*ad_fun.Range());
    CppAD::JacobianRev(ad_fun, delta.template cast<Scalar>().eval(), jac);
    break;
  default:
    MANIF_THROW("Unknown auto differentiation mode.");
    break;
  }

  MANIF_ASSERT(LieGroup::RepSize * LieGroup::DoF == jac.size());

  return Eigen::Map<Jacobian>(jac.data());
}

} // namespace manif

#endif // _MANIF_MANIF_CPPAD_LOCAL_PARAMETRIZATION_H_
