#ifndef _MANIF_MANIF_CERES_OBJECTIVE_H_
#define _MANIF_MANIF_CERES_OBJECTIVE_H_

#include <Eigen/Core>

namespace manif {

template <typename _LieGroup>
class CeresObjectiveFunctor
{
  using LieGroup = _LieGroup;
  using Tangent  = typename _LieGroup::Tangent;

  template <typename _Scalar>
  using LieGroupTemplate = typename LieGroup::template LieGroupTemplate<_Scalar>;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND_TYPE(LieGroup)

  template <typename... Args>
  CeresObjectiveFunctor(Args&&... args)
    : target_state_(std::forward<Args>(args)...)
  {
    //
  }

  CeresObjectiveFunctor(const LieGroup& target_state,
                        const double weight = 1)
    : weight_(weight)
    , target_state_(target_state)
  {
    //
  }

  virtual ~CeresObjectiveFunctor() = default;

  template <typename T>
  bool operator()(const T* const state_raw, T* residuals_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> state(state_raw);

    residuals_raw[0] = (target_state_.template cast<T>() - state).
                          coeffs().norm() * T(weight_);

    /// @todo
    ///
    /// Jacobian G = q.rjac().transpose() * q.rjac();
    /// residual = (q.coeffs().transpose() * G * q.coeffs());
    ///
    /// or
    ///
    /// residual = (q.coeffs().transpose() * W * q.coeffs());

//    std::cout << "State:"
//              << state_raw[0] << "," << state_raw[1]
//              << "\n";

//    std::cout << "Target:"
//              << target_state_.coeffs().transpose()
//              << "\n";

//    std::cout << "residual: " << residuals_raw[0] << "\n";

    return true;
  }

  LieGroup getTargetState() const;
  void setTargetState(const LieGroup& target_state) const;

  inline void weight(const double weight) { weight_ = weight; }
  inline double weight() const noexcept { return weight_; }

protected:

  double weight_ = 1;
  LieGroup target_state_;
};

template <typename _LieGroup>
typename CeresObjectiveFunctor<_LieGroup>::LieGroup
CeresObjectiveFunctor<_LieGroup>::getTargetState() const
{
  return target_state_;
}

template <typename _LieGroup>
void CeresObjectiveFunctor<_LieGroup>::setTargetState(
    const LieGroup& target_state) const
{
  target_state_ = target_state;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_OBJECTIVE_H_ */
