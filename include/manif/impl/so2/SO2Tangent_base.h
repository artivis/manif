#ifndef _MANIF_MANIF_SO2TANGENT_BASE_H_
#define _MANIF_MANIF_SO2TANGENT_BASE_H_

#include "manif/impl/so2/SO2_properties.h"
#include "manif/impl/tangent_base.h"

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

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::coeffs;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SO2TangentBase)

public:

  MANIF_TANGENT_ML_ASSIGN_OP(SO2TangentBase)

  // Tangent common API

  /**
   * @brief Hat operator of SO2.
   * @return An element of the Lie algebra so2 (skew-symmetric matrix).
   * @note See Eqs. (112, 113).
   */
  LieAlg hat() const;

  /**
   * @brief Get the SO2 element.
   * @param[out] -optional- J_m_t Jacobian of the SO2 element wrt this.
   * @return The SO2 element.
   * @note This is the exp() map with the argument in vector form.
   * @note See Eqs. (114, 116) and Eq. (126).
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
   * @brief Get the right Jacobian of SO2.
   * @note See Eq. (126).
   */
  Jacobian rjac() const;

  /**
   * @brief Get the left Jacobian of SO2.
   * @note See Eq. (126).
   */
  Jacobian ljac() const;

  /**
   * @brief Get the inverse of the right Jacobian of SO2.
   * @note See Eq. (126).
   * @see rjac.
   */
  Jacobian rjacinv() const;

  /**
   * @brief Get the inverse of the right Jacobian of SO2.
   * @note See Eq. (126).
   * @see ljac.
   */
  Jacobian ljacinv() const;

  /**
   * @brief
   * @return
   */
  Jacobian smallAdj() const;

  // SO2Tangent specific API

  //const Scalar& angle() const;

  //! @brief Get the angle (rad.).
  Scalar angle() const;
};

template <typename _Derived>
typename SO2TangentBase<_Derived>::LieGroup
SO2TangentBase<_Derived>::exp(OptJacobianRef J_m_t) const
{
  using std::cos;
  using std::sin;

  if (J_m_t)
  {
    (*J_m_t) = rjac();
  }

  return LieGroup(cos(coeffs()(0)), sin(coeffs()(0)));
}

template <typename _Derived>
typename SO2TangentBase<_Derived>::LieGroup
SO2TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename SO2TangentBase<_Derived>::LieAlg
SO2TangentBase<_Derived>::hat() const
{
  return (LieAlg() <<
    Scalar(0)          , Scalar(-coeffs()(0)),
    Scalar(coeffs()(0)), Scalar(0)            ).finished();
}

template <typename _Derived>
typename SO2TangentBase<_Derived>::Jacobian
SO2TangentBase<_Derived>::rjac() const
{
  static const Jacobian Jr = Jacobian::Constant(Scalar(1));
  return Jr;
}

template <typename _Derived>
typename SO2TangentBase<_Derived>::Jacobian
SO2TangentBase<_Derived>::ljac() const
{
  static const Jacobian Jl = Jacobian::Constant(Scalar(1));
  return Jl;
}

template <typename _Derived>
typename SO2TangentBase<_Derived>::Jacobian
SO2TangentBase<_Derived>::rjacinv() const
{
  return rjac();
}

template <typename _Derived>
typename SO2TangentBase<_Derived>::Jacobian
SO2TangentBase<_Derived>::ljacinv() const
{
  return ljac();
}

template <typename _Derived>
typename SO2TangentBase<_Derived>::Jacobian
SO2TangentBase<_Derived>::smallAdj() const
{
  static const Jacobian smallAdj = Jacobian::Zero();
  return smallAdj;
}

// SO2Tangent specific API

//template <typename _Derived>
//const typename SO2TangentBase<_Derived>::Scalar&
//SO2TangentBase<_Derived>::angle() const
//{
//  return coeffs().x();
//}

template <typename _Derived>
typename SO2TangentBase<_Derived>::Scalar
SO2TangentBase<_Derived>::angle() const
{
  return coeffs()(0);
}

namespace internal {

/**
 * @brief Generator specialization for SO2TangentBase objects.
 * E = | 0 -1 |
 *     | 1  0 |
 */
template <typename Derived>
struct GeneratorEvaluator<SO2TangentBase<Derived>>
{
  static typename SO2TangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    MANIF_CHECK(i==0,
                "Index i must be 0!",
                invalid_argument);

    const static typename SO2TangentBase<Derived>::LieAlg E0 =
        skew(typename SO2TangentBase<Derived>::Scalar(1));

    return E0;
  }
};

//! @brief Random specialization for SO2TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<SO2TangentBase<Derived>>
{
  static void run(SO2TangentBase<Derived>& m)
  {
    // in [-1,1]  /  in [-PI,PI]
    m.coeffs().setRandom() *= MANIF_PI;
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_BASE_H_ */
