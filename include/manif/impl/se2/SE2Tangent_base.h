#ifndef _MANIF_MANIF_SE2TANGENT_BASE_H_
#define _MANIF_MANIF_SE2TANGENT_BASE_H_

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the SE2 tangent.
 * @note See Appendix C.
 */
template <typename _Derived>
struct SE2TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SE2TangentBase<_Derived>;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SE2TangentBase)

public:

  MANIF_TANGENT_TYPEDEF

  MANIF_TANGENT_API

  using Base::coeffs;
  using Base::data;

  MANIF_TANGENT_ML_ASSIGN_OP(SE2TangentBase)
  MANIF_TANGENT_OPERATOR

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref exp instead.
   */
  MANIF_DEPRECATED
  LieGroup retract(OptJacobianRef J_m_t = {}) const;

  //! @brief Get the x component of the translational part.
  Scalar x() const;
  //! @brief Get the y component of the translational part.
  Scalar y() const;
  //! @brief Get the rotational part.
  Scalar angle() const;
};

template <typename _Derived>
typename SE2TangentBase<_Derived>::LieGroup
SE2TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

// SE2Tangent specific API

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::angle() const
{
  return coeffs().z();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE2_BASE_H_ */
