#ifndef _MANIF_MANIF_RN_BASE_H_
#define _MANIF_MANIF_RN_BASE_H_

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the Rn group.
 * @note See Appendix E.
 */
template <typename _Derived>
struct RnBase : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = RnBase<_Derived>;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(RnBase)

public:

  MANIF_GROUP_TYPEDEF
  using Transformation = typename internal::traits<_Derived>::Transformation;

  MANIF_GROUP_API

  using Base::coeffs;
  using Base::data;

  MANIF_GROUP_ML_ASSIGN_OP(RnBase)
  MANIF_GROUP_OPERATOR

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref log instead.
   */
  MANIF_DEPRECATED
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief Get the transformation matrix (2D isometry).
   * @note T = | 0 t |
   *           | 0 1 |
   */
  Transformation transform() const;
};

template <typename _Derived>
typename RnBase<_Derived>::Transformation
RnBase<_Derived>::transform() const
{
  Transformation T(Transformation::Identity());
  T.template topRightCorner<Dim,1>() = coeffs();
  return T;
}

} // namespace manif

#endif // _MANIF_MANIF_RN_BASE_H_
