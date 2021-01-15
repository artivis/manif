#ifndef _MANIF_MANIF_RNTANGENT_BASE_H_
#define _MANIF_MANIF_RNTANGENT_BASE_H_

#include "manif/impl/rn/Rn_properties.h"
#include "manif/impl/tangent_base.h"

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

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR
  using Base::coeffs;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(RnTangentBase)

public:

  MANIF_TANGENT_ML_ASSIGN_OP(RnTangentBase)

  // Tangent common API

  /**
   * @brief Hat operator of Rn.
   * @return An element of the Lie algebra rn.
   * @note See Appendix E.
   */
  LieAlg hat() const;

  /**
   * @brief Get the Rn element.
   * @param[out] -optional- J_m_t Jacobian of the Rn element wrt this.
   * @return The Rn element.
   * @note This is the exp() map with the argument in vector form.
   * @note See Eqs. (184) and Eq. (191).
   */
  LieGroup exp(OptJacobianRef J_m_t = {}) const;

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref exp instead.
   */
  MANIF_DEPRECATED
  LieGroup retract(OptJacobianRef J_m_t = {}) const;

  /**
   * @brief Get the right Jacobian of Rn.
   * @note See Eq. (191).
   */
  Jacobian rjac() const;

  /**
   * @brief Get the left Jacobian of Rn.
   * @note See Eq. (191).
   */
  Jacobian ljac() const;

  /**
   * @brief Get the inverse of the right Jacobian of Rn.
   * @note See Eq. (191).
   * @see rjac.
   */
  Jacobian rjacinv() const;

  /**
   * @brief Get the inverse of the right Jacobian of Rn.
   * @note See Eq. (191).
   * @see ljac.
   */
  Jacobian ljacinv() const;

  /**
   * @brief
   * @return
   */
  Jacobian smallAdj() const;

  // RnTangent specific API
};

template <typename _Derived>
typename RnTangentBase<_Derived>::LieGroup
RnTangentBase<_Derived>::exp(OptJacobianRef J_m_t) const
{
  if (J_m_t)
  {
    J_m_t->setIdentity();
  }

  return LieGroup(coeffs());
}

template <typename _Derived>
typename RnTangentBase<_Derived>::LieGroup
RnTangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename RnTangentBase<_Derived>::LieAlg
RnTangentBase<_Derived>::hat() const
{
  LieAlg t_hat = LieAlg::Constant(0);
  t_hat.template topRightCorner<Dim, 1>() = coeffs();
  return t_hat;
}

template <typename _Derived>
typename RnTangentBase<_Derived>::Jacobian
RnTangentBase<_Derived>::rjac() const
{
  static const Jacobian Jr = Jacobian::Identity();
  return Jr;
}

template <typename _Derived>
typename RnTangentBase<_Derived>::Jacobian
RnTangentBase<_Derived>::ljac() const
{
  static const Jacobian Jl = Jacobian::Identity();
  return Jl;
}

template <typename _Derived>
typename RnTangentBase<_Derived>::Jacobian
RnTangentBase<_Derived>::rjacinv() const
{
  return rjac();
}

template <typename _Derived>
typename RnTangentBase<_Derived>::Jacobian
RnTangentBase<_Derived>::ljacinv() const
{
  return ljac();
}

template <typename _Derived>
typename RnTangentBase<_Derived>::Jacobian
RnTangentBase<_Derived>::smallAdj() const
{
  static const Jacobian smallAdj = Jacobian::Constant(0);
  return smallAdj;
}

// RnTangent specific API

namespace internal {

/**
 * @brief Generator specialization for RnTangentBase objects.
 */
template <typename Derived>
struct GeneratorEvaluator<RnTangentBase<Derived>>
{
  static typename RnTangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    MANIF_CHECK(i<RnTangentBase<Derived>::DoF,
                "Index i must less than DoF!",
                invalid_argument);

    using LieAlg = typename RnTangentBase<Derived>::LieAlg;

    LieAlg Ei = LieAlg::Constant(0);

    Ei(i, RnTangentBase<Derived>::DoF) = 1;

    return Ei;
  }
};

//! @brief Random specialization for RnTangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<RnTangentBase<Derived>>
{
  static void run(RnTangentBase<Derived>& m)
  {
    m.coeffs().setRandom();
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_RNTANGENT_BASE_H_
