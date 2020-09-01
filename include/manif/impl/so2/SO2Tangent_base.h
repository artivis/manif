#ifndef _MANIF_MANIF_SO2TANGENT_BASE_H_
#define _MANIF_MANIF_SO2TANGENT_BASE_H_

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the SO2 tangent.
 * @note See Appendix A.
 */
template <typename _Derived>
struct SO2TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SO2TangentBase<_Derived>;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SO2TangentBase)

public:

  MANIF_TANGENT_TYPEDEF

  MANIF_TANGENT_API

  using Base::coeffs;
  using Base::data;

  MANIF_TANGENT_ML_ASSIGN_OP(SO2TangentBase)
  MANIF_TANGENT_OPERATOR

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref exp instead.
   */
  MANIF_DEPRECATED
  LieGroup retract(OptJacobianRef J_m_t = {}) const;

  //const Scalar& angle() const;

  //! @brief Get the angle (rad.).
  Scalar angle() const;
};

template <typename _Derived>
typename SO2TangentBase<_Derived>::LieGroup
SO2TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename SO2TangentBase<_Derived>::Scalar
SO2TangentBase<_Derived>::angle() const
{
  return coeffs()(0);
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_BASE_H_ */
