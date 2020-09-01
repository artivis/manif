#ifndef _MANIF_MANIF_SO3TANGENT_BASE_H_
#define _MANIF_MANIF_SO3TANGENT_BASE_H_

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the SO3 tangent.
 * @note See Appendix B.
 */
template <typename _Derived>
struct SO3TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SO3TangentBase<_Derived>;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SO3TangentBase)

public:

  MANIF_TANGENT_TYPEDEF

  MANIF_TANGENT_API

  using Base::coeffs;
  using Base::data;

  MANIF_TANGENT_ML_ASSIGN_OP(SO3TangentBase)
  MANIF_TANGENT_OPERATOR

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref exp instead.
   */
  MANIF_DEPRECATED
  LieGroup retract(OptJacobianRef J_m_t = {}) const;

  //! @brief
  Scalar x() const;
  //! @brief
  Scalar y() const;
  //! @brief
  Scalar z() const;
};

template <typename _Derived>
typename SO3TangentBase<_Derived>::LieGroup
SO3TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Scalar
SO3TangentBase<_Derived>::x() const
{
  return coeffs()(0);
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Scalar
SO3TangentBase<_Derived>::y() const
{
  return coeffs()(1);
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Scalar
SO3TangentBase<_Derived>::z() const
{
  return coeffs()(2);
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3TANGENT_BASE_H_ */
