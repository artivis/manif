#ifndef _MANIF_MANIF_RxSO2TANGENT_BASE_H_
#define _MANIF_MANIF_RxSO2TANGENT_BASE_H_

#include "manif/impl/rxso2/RxSO2_properties.h"
#include "manif/impl/tangent_base.h"

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the RxSO2 tangent.
 * @note See Appendix B.
 */
template <typename _Derived>
struct RxSO2TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = RxSO2TangentBase<_Derived>;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  // Tangent common API

  using Base::coeffs;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(RxSO2TangentBase)

public:

  MANIF_TANGENT_ML_ASSIGN_OP(RxSO2TangentBase)

  /**
   * @brief Hat operator of RxSO2.
   * @return An element of the Lie algebra rxso3.
   */
  LieAlg hat() const;

  /**
   * @brief Get the RxSO2 element.
   * @param[out] -optional- J_m_t Jacobian of the RxSO2 element wrt this.
   * @return The RxSO2 element.
   * @note This is the exp() map with the argument in vector form.
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
   * Get the right Jacobian of RxSO2.
   */
  Jacobian rjac() const;

  /**
   * Get the left Jacobian of RxSO2.
   */
  Jacobian ljac() const;

  /**
   * Get the inverse of the right Jacobian of RxSO2.
   * @see rjac.
   */
  Jacobian rjacinv() const;

  /**
   * Get the inverse of the left Jacobian of RxSO2.
   * @see ljac.
   */
  Jacobian ljacinv() const;

  /**
   * @brief
   * @return
   */
  Jacobian smallAdj() const;

  // RxSO2Tangent specific API

  //! @brief
  Scalar sigma() const;
  Scalar angle() const;
};

template <typename _Derived>
typename RxSO2TangentBase<_Derived>::LieGroup
RxSO2TangentBase<_Derived>::exp(OptJacobianRef J_m_t) const
{
  using std::cos;
  using std::sin;
  using std::exp;

  if (J_m_t) {
    J_m_t->setIdentity();
  }

  return LieGroup(cos(angle()), sin(angle()), exp(sigma()));
}

template <typename _Derived>
typename RxSO2TangentBase<_Derived>::LieGroup
RxSO2TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename RxSO2TangentBase<_Derived>::Jacobian
RxSO2TangentBase<_Derived>::rjac() const
{
  return Jacobian::Identity();
}

template <typename _Derived>
typename RxSO2TangentBase<_Derived>::Jacobian
RxSO2TangentBase<_Derived>::ljac() const
{
  return Jacobian::Identity();
}

template <typename _Derived>
typename RxSO2TangentBase<_Derived>::Jacobian
RxSO2TangentBase<_Derived>::rjacinv() const
{
  return Jacobian::Identity();
}

template <typename _Derived>
typename RxSO2TangentBase<_Derived>::Jacobian
RxSO2TangentBase<_Derived>::ljacinv() const
{
  return Jacobian::Identity();
}

template <typename _Derived>
typename RxSO2TangentBase<_Derived>::Jacobian
RxSO2TangentBase<_Derived>::smallAdj() const
{
  return Jacobian::Zero();
}

template <typename _Derived>
typename RxSO2TangentBase<_Derived>::LieAlg
RxSO2TangentBase<_Derived>::hat() const
{
  return sigma() * LieAlg::Identity() + skew(angle());
}

// RxSO2Tangent specifics

template <typename _Derived>
typename RxSO2TangentBase<_Derived>::Scalar
RxSO2TangentBase<_Derived>::sigma() const
{
  return coeffs().y();
}

template <typename _Derived>
typename RxSO2TangentBase<_Derived>::Scalar
RxSO2TangentBase<_Derived>::angle() const
{
  return coeffs().x();
}

namespace internal {

//! @brief Generator specialization for RxSO2TangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<RxSO2TangentBase<Derived>>
{
  static typename RxSO2TangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    using LieAlg = typename RxSO2TangentBase<Derived>::LieAlg;
    using Scalar = typename RxSO2TangentBase<Derived>::Scalar;

    switch (i)
    {
      case 0:
      {
        static const LieAlg E0(
              (LieAlg() << Scalar(0), Scalar(-1),
                           Scalar(1), Scalar( 0)).finished());
        return E0;
      }
      case 1:
      {
        static const LieAlg E1(
              (LieAlg() << Scalar(1), Scalar(0),
                           Scalar(0), Scalar(1)).finished());
        return E1;
      }
      default:
        MANIF_THROW("Index i must be in [0,1]!", invalid_argument);
        break;
    }

    return LieAlg{};
  }
};

//! @brief Random specialization for RxSO2TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<RxSO2TangentBase<Derived>>
{
  static void run(RxSO2TangentBase<Derived>& m)
  {
    m.coeffs().setRandom();     // in [-1, 1]
    m.coeffs()(0) *= MANIF_PI;  // in [-PI, PI]
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_RxSO2TANGENT_BASE_H_
