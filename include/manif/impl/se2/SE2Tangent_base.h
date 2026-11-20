#ifndef _MANIF_MANIF_SE2TANGENT_BASE_H_
#define _MANIF_MANIF_SE2TANGENT_BASE_H_

#include "manif/impl/se2/SE2_properties.h"
#include "manif/impl/tangent_base.h"

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

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::data;
  using Base::coeffs;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SE2TangentBase)

public:

  MANIF_TANGENT_ML_ASSIGN_OP(SE2TangentBase)

  // Tangent common API

  /**
   * @brief Hat operator of SE2.
   * @return An element of the Lie algebra se2 (skew-symmetric matrix).
   * @note See Eq. (153).
   */
  LieAlg hat() const;

  /**
   * @brief Get the SE2 element.
   * @param[out] -optional- J_m_t Jacobian of the SE2 element wrt this.
   * @return The SE2 element.
   * @note This is the exp() map with the argument in vector form.
   * @note See Eqs. (156,158) & Eq. (163).
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
   * @brief Get the right Jacobian of SE2.
   * @note See Eq. (163).
   */
  Jacobian rjac() const;

  /**
   * @brief Get the inverse right Jacobian of SE2.
   */
  Jacobian rjacinv() const;

  /**
   * @brief Get the left Jacobian of SE2.
   * @note See Eq. (164).
   */
  Jacobian ljac() const;

  /**
   * @brief Get the inverse left Jacobian of SE2.
   */
  Jacobian ljacinv() const;

  /**
   * @brief
   * @return
   */
  Jacobian smallAdj() const;

  // SE2Tangent specific API

  //! @brief Get the x component of the translational part.
  Scalar x() const;
  //! @brief Get the y component of the translational part.
  Scalar y() const;
  //! @brief Get the rotational part.
  Scalar angle() const;

// private:

  /**
   * @brief Get the coefficients of the exponential map and of
   * the rotational part of the right and left Jacobians,
   * A = sin(theta) / theta and B = (1 - cos(theta)) / theta.
   */
  static void fillAB(Scalar& A, Scalar& B, const Scalar& theta);

  /**
   * @brief Get the coefficients of the translational part of
   * the right and left Jacobians,
   * C = (1 - cos(theta)) / theta^2 and D = (theta - sin(theta)) / theta^2.
   */
  static void fillCD(Scalar& C, Scalar& D, const Scalar& theta);

  /**
   * @brief Get the coefficients of the inverse right and left Jacobians,
   * E = theta/2 cot(theta/2) and F = (1 - E) / theta.
   */
  static void fillEF(Scalar& E, Scalar& F, const Scalar& theta);
};

template <typename _Derived>
typename SE2TangentBase<_Derived>::LieAlg
SE2TangentBase<_Derived>::hat() const
{
  return ( LieAlg() <<
             Scalar(0), -angle(),   x(),
             angle(),    Scalar(0), y(),
             Scalar(0),  Scalar(0), Scalar(0) ).finished();
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::LieGroup
SE2TangentBase<_Derived>::exp(OptJacobianRef J_m_t) const
{
  using std::abs;
  using std::cos;
  using std::sin;

  const Scalar theta = angle();
  const Scalar cos_theta = cos(theta);
  const Scalar sin_theta = sin(theta);

  Scalar A,  // sin_theta_by_theta
         B;  // one_minus_cos_theta_by_theta

  fillAB(A, B, theta);

  if (J_m_t)
  {
    // Jr
    J_m_t->setIdentity();
    (*J_m_t)(0,0) =  A;
    (*J_m_t)(0,1) =  B;
    (*J_m_t)(1,0) = -B;
    (*J_m_t)(1,1) =  A;

    Scalar C,  // one_minus_cos_theta_by_theta_sq
           D;  // theta_minus_sin_theta_by_theta_sq

    fillCD(C, D, theta);

    (*J_m_t)(0,2) = D * x() - C * y();
    (*J_m_t)(1,2) = D * y() + C * x();
  }

  return LieGroup( A * x() - B * y(),
                   B * x() + A * y(),
                   cos_theta, sin_theta );
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::LieGroup
SE2TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Jacobian
SE2TangentBase<_Derived>::rjac() const
{
//  const Scalar theta = angle();
//  const Scalar cos_theta = cos(theta);
//  const Scalar sin_theta = sin(theta);
//  const Scalar theta_sq = theta * theta;

//  Scalar A,  // sin_theta_by_theta
//         B;  // one_minus_cos_theta_by_theta

//  if (abs(theta) < Constants<Scalar>::eps)
//  {
//    // Taylor approximation
//    A = Scalar(1) - Scalar(1. / 6.) * theta_sq;
//    B = Scalar(.5) * theta - Scalar(1. / 24.) * theta * theta_sq;
//  }
//  else
//  {
//    // Euler
//    A = sin_theta / theta;
//    B = (Scalar(1) - cos_theta) / theta;
//  }

//  Jacobian Jr = Jacobian::Identity();
//  Jr(0,0) =  A;
//  Jr(0,1) =  B;
//  Jr(1,0) = -B;
//  Jr(1,1) =  A;

//  Jr(0,2) = (-y() + theta*x() + y()*cos_theta - x()*sin_theta)/theta_sq;
//  Jr(1,2) = ( x() + theta*y() - x()*cos_theta - y()*sin_theta)/theta_sq;

//  return Jr;

  Jacobian Jr;
  exp(Jr);

  return Jr;
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Jacobian
SE2TangentBase<_Derived>::rjacinv() const
{
  const Scalar theta = angle();

  Scalar E,  // half_theta_cot_half_theta
         F;  // one_minus_half_theta_cot_half_theta_by_theta

  fillEF(E, F, theta);

  Jacobian Jrinv;

  Jrinv(0,0) =  E;
  Jrinv(0,1) = -theta*Scalar(0.5);
  Jrinv(1,0) = -Jrinv(0,1);
  Jrinv(1,1) =  E;

  Jrinv(0,2) = F * x() + y()/Scalar(2);
  Jrinv(1,2) = F * y() - x()/Scalar(2);

  Jrinv(2,0) = Scalar(0);
  Jrinv(2,1) = Scalar(0);
  Jrinv(2,2) = Scalar(1);

  return Jrinv;
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Jacobian
SE2TangentBase<_Derived>::ljac() const
{
  const Scalar theta = angle();

  Scalar A,  // sin_theta_by_theta
         B;  // one_minus_cos_theta_by_theta

  fillAB(A, B, theta);

  Jacobian Jl = Jacobian::Identity();
  Jl(0,0) =  A;
  Jl(0,1) = -B;
  Jl(1,0) =  B;
  Jl(1,1) =  A;

  Scalar C,  // one_minus_cos_theta_by_theta_sq
         D;  // theta_minus_sin_theta_by_theta_sq

  fillCD(C, D, theta);

  Jl(0,2) = D * x() + C * y();
  Jl(1,2) = D * y() - C * x();

  return Jl;
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Jacobian
SE2TangentBase<_Derived>::ljacinv() const
{
  const Scalar theta = angle();

  Scalar E,  // half_theta_cot_half_theta
         F;  // one_minus_half_theta_cot_half_theta_by_theta

  fillEF(E, F, theta);

  Jacobian Jlinv;

  Jlinv(0,0) =  E;
  Jlinv(0,1) =  theta*Scalar(0.5);
  Jlinv(1,0) = -Jlinv(0,1);
  Jlinv(1,1) =  E;

  Jlinv(0,2) = F * x() - y()/Scalar(2);
  Jlinv(1,2) = F * y() + x()/Scalar(2);

  Jlinv(2,0) = Scalar(0);
  Jlinv(2,1) = Scalar(0);
  Jlinv(2,2) = Scalar(1);

  return Jlinv;
}

template <typename _Derived>
void SE2TangentBase<_Derived>::fillAB(
  Scalar& A, Scalar& B, const Scalar& theta)
{
  using std::sin;

  const Scalar theta_sq = theta * theta;

  if (theta_sq < Constants<Scalar>::eps)
  {
    // Taylor approximation
    A = Scalar(1) - Scalar(1. / 6.) * theta_sq;
    B = Scalar(.5) * theta - Scalar(1. / 24.) * theta * theta_sq;
  }
  else
  {
    // Euler
    // 1 - cos(theta) = 2 sin(theta/2)^2 avoids cancellation
    const Scalar sin_half_theta = sin(theta * Scalar(0.5));

    A = sin(theta) / theta;
    B = Scalar(2) * sin_half_theta * sin_half_theta / theta;
  }
}

template <typename _Derived>
void SE2TangentBase<_Derived>::fillCD(
  Scalar& C, Scalar& D, const Scalar& theta)
{
  using std::sin;

  const Scalar theta_sq = theta * theta;
  const Scalar theta_p4 = theta_sq * theta_sq;

  // theta - sin(theta) cancels to O(theta^3) so that D loses
  // precision as eps / theta. The series is thus used up to
  // theta^8 < eps, where its neglected terms are still below eps.
  if (theta_p4 * theta_p4 < Constants<Scalar>::eps)
  {
    // Taylor approximation
    C = Scalar(1. / 2.) - Scalar(1. / 24.) * theta_sq +
        Scalar(1. / 720.) * theta_p4 -
        Scalar(1. / 40320.) * theta_p4 * theta_sq;
    D = theta * (Scalar(1. / 6.) - Scalar(1. / 120.) * theta_sq +
                 Scalar(1. / 5040.) * theta_p4 -
                 Scalar(1. / 362880.) * theta_p4 * theta_sq);
  }
  else
  {
    // 1 - cos(theta) = 2 sin(theta/2)^2 avoids cancellation
    const Scalar sin_half_theta = sin(theta * Scalar(0.5));

    C = Scalar(2) * sin_half_theta * sin_half_theta / theta_sq;
    D = (theta - sin(theta)) / theta_sq;
  }
}

template <typename _Derived>
void SE2TangentBase<_Derived>::fillEF(
  Scalar& E, Scalar& F, const Scalar& theta)
{
  using std::cos;
  using std::sin;

  const Scalar theta_sq = theta * theta;
  const Scalar theta_p4 = theta_sq * theta_sq;

  // 1 - theta/2 cot(theta/2) cancels to O(theta^2) so that F loses
  // precision as eps / theta. The series is thus used up to
  // theta^8 < eps, where its neglected terms are still below eps.
  if (theta_p4 * theta_p4 < Constants<Scalar>::eps)
  {
    // Taylor approximation
    F = theta * (Scalar(1. / 12.) + Scalar(1. / 720.) * theta_sq +
                 Scalar(1. / 30240.) * theta_p4 +
                 Scalar(1. / 1209600.) * theta_p4 * theta_sq);
    E = Scalar(1) - theta * F;
  }
  else
  {
    const Scalar half_theta = theta * Scalar(0.5);

    E = half_theta * cos(half_theta) / sin(half_theta);
    F = (Scalar(1) - E) / theta;
  }
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Jacobian
SE2TangentBase<_Derived>::smallAdj() const
{
  Jacobian smallAdj = Jacobian::Zero();

  smallAdj(0,1) = -angle();
  smallAdj(1,0) =  angle();
  smallAdj(0,2) =  y();
  smallAdj(1,2) = -x();

  return smallAdj;
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

namespace internal {

//! @brief Generator specialization for SE2TangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<SE2TangentBase<Derived>>
{
  static typename SE2TangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    using LieAlg = typename SE2TangentBase<Derived>::LieAlg;
    using Scalar = typename SE2TangentBase<Derived>::Scalar;

    switch (i)
    {
      case 0:
      {
        const static LieAlg E0(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(1),
                             Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E0;
      }
      case 1:
      {
        const static LieAlg E1(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(1),
                             Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E1;
      }
      case 2:
      {
        const static LieAlg E2(
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

//! @brief Inner weight matrix specialization for SE2TangentBase objects.
template <typename Derived>
struct InnerWeightsEvaluator<SE2TangentBase<Derived>>
{
  static typename Derived::InnerWeightsMatrix
  run()
  {
    using InnerWeightsMatrix = typename SE2TangentBase<Derived>::InnerWeightsMatrix;
    using Scalar = typename SE2TangentBase<Derived>::Scalar;

    const static InnerWeightsMatrix W(
      (InnerWeightsMatrix() << Scalar(1), Scalar(0), Scalar(0),
                               Scalar(0), Scalar(1), Scalar(0),
                               Scalar(0), Scalar(0), Scalar(2) ).finished()
    );

    return W;
  }
};

//! @brief Random specialization for SE2TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<SE2TangentBase<Derived>>
{
  static void run(SE2TangentBase<Derived>& m)
  {
    m.coeffs().setRandom();         // in [-1,1]
    m.coeffs().coeffRef(2) *= MANIF_PI; // in [-PI,PI]
  }
};

//! @brief Vee specialization for SE2TangentBase objects.
template <typename Derived>
struct VeeEvaluatorImpl<SE2TangentBase<Derived>> {
  template <typename TL, typename TR>
  static void run(TL& t, const TR& v) {
    t.coeffs() << v(0, 2), v(1, 2), v(1, 0);
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SE2_BASE_H_ */
