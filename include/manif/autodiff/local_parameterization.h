#ifndef _MANIF_MANIF_AUTODIFF_LOCAL_PARAMETRIZATION_H_
#define _MANIF_MANIF_AUTODIFF_LOCAL_PARAMETRIZATION_H_

namespace manif {

template <typename Ad, typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RepSize, Derived::DoF>
autodiffLocalParameterizationJacobian(const manif::LieGroupBase<Derived>& _state) {

  using Scalar = typename Derived::Scalar;
  using LieGroup = typename Derived::template LieGroupTemplate<Ad>;
  using Tangent = typename Derived::Tangent::template TangentTemplate<Ad>;

  using Jac = Eigen::Matrix<Scalar, Derived::RepSize, Derived::DoF>;

  LieGroup state = _state.template cast<Ad>();
  Tangent delta = Tangent::Zero();

  LieGroup state_plus_delta;

  auto f = [](const auto& s, const auto& t){
    return s + t;
  };

  Jac J_so_t = autodiff::jacobian(
    f, autodiff::wrt(delta), autodiff::at(state, delta), state_plus_delta
  );

  MANIF_ASSERT(state.isApprox(state_plus_delta));
  MANIF_ASSERT(Derived::RepSize == J_so_t.rows());
  MANIF_ASSERT(Derived::DoF == J_so_t.cols());

  return J_so_t;
}

} // namespace manif
#endif // _MANIF_MANIF_AUTODIFF_LOCAL_PARAMETRIZATION_H_