#ifndef _MANIF_MANIF_RNTANGENT_BASE_H_
#define _MANIF_MANIF_RNTANGENT_BASE_H_

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the R^n tangent.
 * @note See Appendix E.
 */
template <typename _Derived>
struct RnTangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = RnTangentBase<_Derived>;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(RnTangentBase)

public:

  MANIF_TANGENT_TYPEDEF

  MANIF_TANGENT_API
  using Base::coeffs;
  using Base::data;

  MANIF_TANGENT_ML_ASSIGN_OP(RnTangentBase)
  MANIF_TANGENT_OPERATOR

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref exp instead.
   */
  MANIF_DEPRECATED
  LieGroup retract(OptJacobianRef J_m_t = {}) const;
};

template <typename _Derived>
typename RnTangentBase<_Derived>::LieGroup
RnTangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

} // namespace manif

#endif // _MANIF_MANIF_RNTANGENT_BASE_H_
