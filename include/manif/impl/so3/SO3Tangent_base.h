#ifndef _MANIF_MANIF_SO3TANGENT_BASE_H_
#define _MANIF_MANIF_SO3TANGENT_BASE_H_

#include "manif/impl/so3/SO3_properties.h"
#include "manif/impl/tangent_base.h"

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

public:

  MANIF_TANGENT_PROPERTIES

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  // Tangent common API

  using Base::coeffs;

  SO3TangentBase()  = default;
  ~SO3TangentBase() = default;

  /**
   * @brief Hat operator of SO3.
   * @return An element of the Lie algebra so3 (skew-symmetric matrix).
   * @note See example 3 of the paper.
   */
  LieAlg hat() const;

  /**
   * @brief Get the SO3 element.
   * @param[out] -optional- J_m_t Jacobian of the SO3 element wrt this.
   * @return The SO3 element.
   * @note This is the exp() map with the argument in vector form.
   * @note See Eq. (132) and Eq. (143).
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
   * Get the right Jacobian of SO3.
   * @note See Eq. (143).
   */
  Jacobian rjac() const;

  /**
   * Get the left Jacobian of SO3.
   * @note See Eq. (145).
   */
  Jacobian ljac() const;

  /**
   * Get the inverse of the right Jacobian of SO3.
   * @note See Eq. (144).
   * @see rjac.
   */
  Jacobian rjacinv() const;

  /**
   * Get the inverse of the left Jacobian of SO3.
   * @note See Eq. (146).
   * @see ljac.
   */
  Jacobian ljacinv() const;

  /**
   * @brief
   * @return
   */
  Jacobian smallAdj() const;

  // SO3Tangent specific API

  //! @brief
  Scalar x() const;
  //! @brief
  Scalar y() const;
  //! @brief
  Scalar z() const;
};

template <typename _Derived>
typename SO3TangentBase<_Derived>::LieGroup
SO3TangentBase<_Derived>::exp(OptJacobianRef J_m_t) const
{
  using std::sqrt;
  using std::cos;
  using std::sin;

  const DataType& theta_vec = coeffs();
  const Scalar theta_sq = theta_vec.squaredNorm();
  const Scalar theta    = sqrt(theta_sq);

  if (theta_sq > Constants<Scalar>::eps_s)
  {
    if (J_m_t)
    {
      Jacobian M1, M2;

      const LieAlg W = hat();

      M1.noalias() = (Scalar(1.0) - cos(theta)) / theta_sq * W;
      M2.noalias() = (theta - sin(theta)) / (theta_sq * theta) * (W * W);;

      *J_m_t = Jacobian::Identity() - M1 + M2;
    }

    return LieGroup( Eigen::AngleAxis<Scalar>(theta, theta_vec.normalized()) );
  }
  else
  {
    if (J_m_t)
    {
      *J_m_t = Jacobian::Identity() - Scalar(0.5) * hat();
    }

    return LieGroup(x()/Scalar(2), y()/Scalar(2), z()/Scalar(2), Scalar(1));
  }
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::LieGroup
SO3TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Jacobian
SO3TangentBase<_Derived>::rjac() const
{
  return ljac().transpose();
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Jacobian
SO3TangentBase<_Derived>::ljac() const
{
  using std::sqrt;
  using std::cos;
  using std::sin;

  const Scalar theta_sq = coeffs().squaredNorm();

  const LieAlg W = hat();

  // Small angle approximation
  if (theta_sq <= Constants<Scalar>::eps_s)
    return Jacobian::Identity() - Scalar(0.5) * W;

  const Scalar theta = sqrt(theta_sq); // rotation angle
  Jacobian M1, M2;
  M1.noalias() = (Scalar(1) - cos(theta)) / theta_sq * W;
  M2.noalias() = (theta - sin(theta)) / (theta_sq * theta) * (W * W);

  return Jacobian::Identity() + M1 + M2;
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Jacobian
SO3TangentBase<_Derived>::rjacinv() const
{
  return ljacinv().transpose();
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Jacobian
SO3TangentBase<_Derived>::ljacinv() const
{
  using std::sqrt;
  using std::cos;
  using std::sin;

  const Scalar theta_sq = coeffs().squaredNorm();

  const LieAlg W = hat();

  if (theta_sq <= Constants<Scalar>::eps_s)
    return Jacobian::Identity() + Scalar(0.5) * W;

  const Scalar theta = sqrt(theta_sq); // rotation angle
  Jacobian M;
  M.noalias() = (Scalar(1) / theta_sq -
                 (Scalar(1) + cos(theta)) /
                 (Scalar(2) * theta * sin(theta))) * (W * W);

  return Jacobian::Identity() - Scalar(0.5) * W + M;
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::Jacobian
SO3TangentBase<_Derived>::smallAdj() const
{
  return hat();
}

template <typename _Derived>
typename SO3TangentBase<_Derived>::LieAlg
SO3TangentBase<_Derived>::hat() const
{
  return skew(coeffs());
}

// SO3Tangent specifics

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

namespace internal {

//! @brief Generator specialization for SO3TangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<SO3TangentBase<Derived>>
{
  static typename SO3TangentBase<Derived>::LieAlg
  run(const int i)
  {
    using LieAlg = typename SO3TangentBase<Derived>::LieAlg;
    using Scalar = typename SO3TangentBase<Derived>::Scalar;

    switch (i)
    {
      case 0:
      {
        static const LieAlg E0(
              (LieAlg() << Scalar(0), Scalar(0), Scalar( 0),
                           Scalar(0), Scalar(0), Scalar(-1),
                           Scalar(0), Scalar(1), Scalar( 0) ).finished());
        return E0;
      }
      case 1:
      {
        static const LieAlg E1(
              (LieAlg() << Scalar( 0), Scalar(0), Scalar(1),
                           Scalar( 0), Scalar(0), Scalar(0),
                           Scalar(-1), Scalar(0), Scalar(0) ).finished());
        return E1;
      }
      case 2:
      {
        static const LieAlg E2(
              (LieAlg() << Scalar(0), Scalar(-1), Scalar(0),
                           Scalar(1), Scalar( 0), Scalar(0),
                           Scalar(0), Scalar( 0), Scalar(0) ).finished());
        return E2;
      }
      default:
        MANIF_THROW("Index i must be in [0,2]!", invalid_argument);
        break;
    }

    return LieAlg{};
  }
};

//! @brief Random specialization for SO3TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<SO3TangentBase<Derived>>
{
  static void run(SO3TangentBase<Derived>& m)
  {
    // in [-1,1]  / in [-PI,PI]
    m.coeffs().setRandom() *= MANIF_PI;
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SO3TANGENT_BASE_H_ */
