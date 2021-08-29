#ifndef _MANIF_MANIF_SIM2TANGENT_BASE_H_
#define _MANIF_MANIF_SIM2TANGENT_BASE_H_

#include "manif/impl/sim2/Sim2_properties.h"
#include "manif/impl/tangent_base.h"
#include "manif/impl/rxso2/RxSO2Tangent_map.h"

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the Sim2 tangent.
 * @note See Appendix D of the paper.
 */
template <typename _Derived>
struct Sim2TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = Sim2TangentBase<_Derived>;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using LinBlock = typename DataType::template FixedSegmentReturnType<2>::Type;
  using ConstLinBlock = typename DataType::template ConstFixedSegmentReturnType<2>::Type;

  using Base::data;
  using Base::coeffs;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(Sim2TangentBase)

public:

  MANIF_TANGENT_ML_ASSIGN_OP(Sim2TangentBase)

  // Tangent common API

  /**
   * @brief Hat operator of Sim2.
   * @return An element of the Lie algebra sim2.
   */
  LieAlg hat() const;

  /**
   * @brief Get the Sim2 element.
   * @param[out] -optional- J_m_t Jacobian of the Sim2 element wrt this.
   * @return The Sim2 element.
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
   * @brief Get the right Jacobian of Sim2.
   */
  Jacobian rjac() const;

  /**
   * @brief Get the left Jacobian of Sim2.
   */
  Jacobian ljac() const;

  /**
   * @brief Get the inverse right Jacobian of Sim2.
   */
  Jacobian rjacinv() const;

  /**
   * @brief Get the inverse left Jacobian of Sim2.
   */
  Jacobian ljacinv() const;

  /**
   * @brief
   * @return
   */
  Jacobian smallAdj() const;

  // Sim2Tangent specific API

  //! @brief Get the x component of the translational part.
  Scalar x() const;
  //! @brief Get the y component of the translational part.
  Scalar y() const;
  //! @brief Get the rotational part.
  Scalar angle() const;
  //! @brief Get the scaling part.
  Scalar sigma() const;

  //! @brief Get the linear part.
  LinBlock lin();
  const ConstLinBlock lin() const;

// protected:

  const Eigen::Map<const RxSO2Tangent<Scalar>> asRxSO2() const;
  Eigen::Map<RxSO2Tangent<Scalar>> asRxSO2();

  static void fillW(
    Eigen::Ref<Eigen::Matrix<Scalar, 2, 2>> W, const Scalar phi, const Scalar sigma
  );
  static void fillWinv(
    Eigen::Ref<Eigen::Matrix<Scalar, 2, 2>> W, const Scalar phi, const Scalar sigma
  );
  static Eigen::Matrix<Scalar, 2, 2> computeW(const Scalar phi, const Scalar sigma);
  static Eigen::Matrix<Scalar, 2, 2> computeWinv(const Scalar phi, const Scalar sigma);
};

template <typename _Derived>
typename Sim2TangentBase<_Derived>::LieGroup
Sim2TangentBase<_Derived>::exp(OptJacobianRef J_m_t) const
{
  if (J_m_t) {
    *J_m_t = rjac();
  }

  return LieGroup(computeW(angle(), sigma())*lin(), asRxSO2().exp());
}

template <typename _Derived>
typename Sim2TangentBase<_Derived>::LieGroup
Sim2TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename Sim2TangentBase<_Derived>::LieAlg
Sim2TangentBase<_Derived>::hat() const
{
  LieAlg hat;
  hat.template topLeftCorner<2, 2>() = asRxSO2().hat();
  hat.template topRightCorner<2, 1>() = lin();
  hat.template bottomRows<1>().setZero();

  return hat;
}

template <typename _Derived>
typename Sim2TangentBase<_Derived>::Jacobian
Sim2TangentBase<_Derived>::rjac() const
{
  /// this is
  ///       rjac =  | W(-tau)     ?       ? |
  ///               | 0       rjac(SO2)   0 |
  ///               | 0           0       1 |
  ///
  /// with rjac(SO2) = 1
  ///
  /// considering vee(log(g)) = (v;w;s)

  Jacobian Jr = Jacobian::Zero();

  fillW(Jr.template topLeftCorner<2,2>(), -angle(), -sigma());

  // Jr.template topRightCorner<2,2>() =

  Jr.template bottomLeftCorner<2,2>().setZero();
  Jr.template bottomRightCorner<2,2>().setIdentity();

  return Jr;
}

template <typename _Derived>
typename Sim2TangentBase<_Derived>::Jacobian
Sim2TangentBase<_Derived>::ljac() const
{
  /// this is
  ///       rjac =  |      W         Q       -lin |
  ///               |      0     rjac(SO3)     0  |
  ///               |      0         0         1  |
  ///
  /// considering vee(log(g)) = (v;w;s)

  Jacobian Jl;
  // fillW(Jl.template topLeftCorner<3,3>(), ang(), coeffs()(6),
  //  {}, Jl.template bottomLeftCorner<3,3>());
  // Jl.template topRightCorner<3,1>() = Jl.template bottomLeftCorner<3,3>()*lin();
  // //  {}, Jl.template topRightCorner<3,1>());


  // fillQ( Jl.template block<3,3>(0,3), coeffs() );
  // // Jl.template block<3,3>(0,3) = computeW(ang(), coeffs()(6));
  // // Jl.template topRightCorner<3,1>() = lin();

  // Jl.template block<3,3>(3,3) = asSO3().rjac();

  // Jl.template bottomLeftCorner<4,3>().setZero();
  // Jl.template block<3,1>(3,6).setZero();
  // Jl.template block<1,3>(6,3).setZero();
  // Jl(6, 6) = Scalar(1);

  return Jl;
}

template <typename _Derived>
typename Sim2TangentBase<_Derived>::Jacobian
Sim2TangentBase<_Derived>::rjacinv() const
{
  return rjac().inverse();
}

template <typename _Derived>
typename Sim2TangentBase<_Derived>::Jacobian
Sim2TangentBase<_Derived>::ljacinv() const
{
  return ljac().inverse();
}

template <typename _Derived>
typename Sim2TangentBase<_Derived>::Jacobian
Sim2TangentBase<_Derived>::smallAdj() const
{
  /// this is
  ///       ad(g) = |  Omega+I.s [v]x -v |
  ///               |      0      w    0 |
  ///               |      0      0    0 |
  ///
  /// with Omega = [w]x
  ///
  /// considering vee(log(g)) = (v;w;s)

  Jacobian smallAdj;

  smallAdj(0,0) =  sigma();
  smallAdj(1,1) =  sigma();
  smallAdj(1,0) =  angle();
  smallAdj(0,1) = -angle();
  smallAdj(0,2) =  y();
  smallAdj(1,2) = -x();
  smallAdj(0,3) = -x();
  smallAdj(1,3) = -y();

  smallAdj.template bottomRows<2>().setZero();

  smallAdj(2,2) = angle();

  return smallAdj;
}

// Sim2Tangent specific API

template <typename _Derived>
typename Sim2TangentBase<_Derived>::LinBlock
Sim2TangentBase<_Derived>::lin()
{
  return coeffs().template head<2>();
}

template <typename _Derived>
const typename Sim2TangentBase<_Derived>::ConstLinBlock
Sim2TangentBase<_Derived>::lin() const
{
  return coeffs().template head<2>();
}

template <typename _Derived>
const Eigen::Map<const RxSO2Tangent<typename Sim2TangentBase<_Derived>::Scalar>>
Sim2TangentBase<_Derived>::asRxSO2() const
{
  return Eigen::Map<const RxSO2Tangent<Scalar>>(coeffs().data()+2);
}

template <typename _Derived>
Eigen::Map<RxSO2Tangent<typename Sim2TangentBase<_Derived>::Scalar>>
Sim2TangentBase<_Derived>::asRxSO2()
{
  return Eigen::Map<RxSO2Tangent<Scalar>>(coeffs().data()+2);
}

template <typename _Derived>
void Sim2TangentBase<_Derived>::fillW(
  Eigen::Ref<Eigen::Matrix<Scalar, 2, 2>> W, const Scalar phi, const Scalar sigma
) {
  using SO2LieAlg = Eigen::Matrix<Scalar, 2, 2>;
  using std::abs;
  using std::cos;
  using std::sin;
  using std::exp;
  using std::sqrt;

  const SO2LieAlg I = SO2LieAlg::Identity();
  const Scalar half(0.5);
  const Scalar one(1);
  const Scalar two(2);
  const Scalar three(3);
  const Scalar six(6);
  const Scalar twenty_four(24);

  const Scalar theta = phi;
  const Scalar theta_sq = theta*theta;
  const Scalar scale = exp(sigma);

  const SO2LieAlg Phi = skew(theta);
  const SO2LieAlg Phi2 = Phi * Phi;

  // Below we make use of Taylor series and adapt the order
  // up to the point where both terms (theta & sigma) appears
  // so that auto-diff can run on those.

  // used matlab syms and https://www.geogebra.org/m/Ehnz3hGb

  // @todo there is some comput. opti. to do here.

  Scalar A, B, C;
  if (abs(sigma) < Constants<Scalar>::eps) {
    // Taylor second order at sigma=0
    C = one + half*sigma;

    if (abs(theta_sq) < Constants<Scalar>::eps) {
      // Taylor first order at sigma=0 theta=1
      // Note that the derivative does not exist when sigma=0 theta=0

      const Scalar cos_one = cos(one);

      A = one - cos(one) + (theta-one)*(-two+sin(one)+two*cos_one) -
          sqrt(two) * sigma * cos(MANIF_PI_4 + one);

      // Taylor first order at sigma=1 theta=1
      // Note that the derivative does not exist when
      // sigma=0 theta=0 nor at sigma=0 theta=1

      const Scalar e = exp(one);
      const Scalar sin_one = sin(one);

      B = -two + sigma + e*(three-cos_one-two*sin_one+theta*(cos_one-two)) +
          half*e*(three*theta*sin(one)-sigma*cos_one) + half*theta;
    } else {
      // Taylor second order at sigma=0
      const Scalar cos_theta = cos(theta);
      const Scalar sin_theta = sin(theta);
      const Scalar theta_c = theta_sq * theta;

      A = (one-cos_theta-sigma*cos_theta) / theta_sq + (sigma*sin_theta)/theta_c;
      B = (two+sigma)/theta_sq -
          (sin_theta-sigma*sin_theta)/theta_c +
          (sigma-sigma*cos_theta)/(theta_sq*theta_sq);
    }
  } else {

    C = (scale - one) / sigma;

    if (abs(theta_sq) < Constants<Scalar>::eps) {
      // Taylor third order at theta=0
      const Scalar sigma_scale = sigma*scale;
      const Scalar sigma_sq = sigma * sigma;
      const Scalar sigma_c = sigma_sq * sigma;
      const Scalar sigma_4 = sigma_c * sigma;

      A = (
        six*sigma_sq*(one-scale+sigma_scale) +
        theta_sq*(six*(scale-one) + sigma_scale*(three*sigma-six-sigma*sigma_scale))
      ) / six*sigma_4;

      B = (
        sigma_4*scale*(Scalar(12)-theta_sq) +
        sigma_c*scale*(Scalar(4)*theta_sq-twenty_four) +
        sigma_sq*(scale*(twenty_four-Scalar(8)*theta_sq)-twenty_four) +
        theta_sq*(twenty_four*(sigma_scale-scale-one))
      ) / twenty_four*sigma_4*sigma;
    } else {
      // Motion Interpolation in SIM(3)
      // C. Allen-blanchette, S. Leonardos & J. Gallier

      const Scalar a = scale * sin(theta);
      const Scalar b = scale * cos(theta);
      const Scalar c = theta_sq + sigma * sigma;
      A = (a * sigma + theta * (one - b)) / (theta * c);
      // B = (C - ((b - one) * sigma + a * theta) / c) / theta_sq;
      B = C / theta_sq - a / (theta * c) - (sigma * (b - one)) / (theta_sq * c);
    }
  }

  W = A * Phi + B * Phi2 + C * I;
}

template <typename _Derived>
void Sim2TangentBase<_Derived>::fillWinv(
  Eigen::Ref<Eigen::Matrix<Scalar, 2, 2>> W, const Scalar phi, const Scalar sigma
) {
  using SO2LieAlg = Eigen::Matrix<Scalar, 2, 2>;
  using std::abs;
  using std::cos;
  using std::sin;
  using std::exp;
  using std::sqrt;

  const SO2LieAlg I = SO2LieAlg::Identity();
  const Scalar half(0.5);
  const Scalar one(1);
  const Scalar two(2);
  const Scalar three(3);
  const Scalar six(6);
  const Scalar twenty_four(24);

  const Scalar theta = phi;
  const Scalar theta_sq = theta*theta;
  const Scalar scale = exp(sigma);

  const SO2LieAlg Phi = skew(theta);
  const SO2LieAlg Phi2 = Phi * Phi;

  // Below we make use of Taylor series and adapt the order
  // up to the point where both terms (theta & sigma) appears
  // so that auto-diff can run on those.

  // used matlab syms and https://www.geogebra.org/m/Ehnz3hGb

  // @todo there is some comput. opti. to do here.

  Scalar A, B, C;
  if (abs(sigma) < Constants<Scalar>::eps) {
    // Taylor second order at sigma=0
    C = one + half*sigma;

    if (abs(theta_sq) < Constants<Scalar>::eps) {
      // Taylor first order at sigma=0 theta=1
      // Note that the derivative does not exist when sigma=0 theta=0

      const Scalar cos_one = cos(one);

      A = one - cos(one) + (theta-one)*(-two+sin(one)+two*cos_one) -
          sqrt(two) * sigma * cos(MANIF_PI_4 + one);

      // Taylor first order at sigma=1 theta=1
      // Note that the derivative does not exist when
      // sigma=0 theta=0 nor at sigma=0 theta=1

      const Scalar e = exp(one);
      const Scalar sin_one = sin(one);

      B = -two + sigma + e*(three-cos_one-two*sin_one+theta*(cos_one-two)) +
          half*e*(three*theta*sin(one)-sigma*cos_one) + half*theta;
    } else {
      // Taylor second order at sigma=0
      const Scalar cos_theta = cos(theta);
      const Scalar sin_theta = sin(theta);
      const Scalar theta_c = theta_sq * theta;

      A = (one-cos_theta-sigma*cos_theta) / theta_sq + (sigma*sin_theta)/theta_c;
      B = (two+sigma)/theta_sq -
          (sin_theta-sigma*sin_theta)/theta_c +
          (sigma-sigma*cos_theta)/(theta_sq*theta_sq);
    }
  } else {

    C = sigma / (scale - one);

    if (abs(theta_sq) < Constants<Scalar>::eps) {
      // Taylor third order at theta=0
      const Scalar sigma_scale = sigma*scale;
      const Scalar sigma_sq = sigma * sigma;
      const Scalar sigma_c = sigma_sq * sigma;
      const Scalar sigma_4 = sigma_c * sigma;

      A = (
        six*sigma_sq*(one-scale+sigma_scale) +
        theta_sq*(six*(scale-one) + sigma_scale*(three*sigma-six-sigma*sigma_scale))
      ) / six*sigma_4;

      B = (
        sigma_4*scale*(Scalar(12)-theta_sq) +
        sigma_c*scale*(Scalar(4)*theta_sq-twenty_four) +
        sigma_sq*(scale*(twenty_four-Scalar(8)*theta_sq)-twenty_four) +
        theta_sq*(twenty_four*(sigma_scale-scale-one))
      ) / twenty_four*sigma_4*sigma;
    } else {
      // Motion Interpolation in SIM(3)
      // C. Allen-blanchette, S. Leonardos & J. Gallier

      const Scalar a = scale * sin(theta);
      const Scalar b = scale * cos(theta);
      const Scalar c = theta_sq + sigma * sigma;
      A = (a * sigma + theta * (one - b)) / (theta * c);
      // B = (C - ((b - one) * sigma + a * theta) / c) / theta_sq;
      B = C / theta_sq - a / (theta * c) - (sigma * (b - one)) / (theta_sq * c);
    }
  }

  W = A * Phi + B * Phi2 + C * I;
}

template <typename _Derived>
Eigen::Matrix<typename Sim2TangentBase<_Derived>::Scalar, 2, 2>
Sim2TangentBase<_Derived>::computeW(const Scalar phi, const Scalar sigma) {
  Eigen::Matrix<Scalar, 2, 2> W;
  fillW(W, phi, sigma);
  return W;
}

template <typename _Derived>
Eigen::Matrix<typename Sim2TangentBase<_Derived>::Scalar, 2, 2>
Sim2TangentBase<_Derived>::computeWinv(const Scalar phi, const Scalar sigma) {
  Eigen::Matrix<Scalar, 2, 2> Winv;
  fillW(Winv, phi, sigma);
  return Winv;
}

template <typename _Derived>
typename Sim2TangentBase<_Derived>::Scalar
Sim2TangentBase<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename Sim2TangentBase<_Derived>::Scalar
Sim2TangentBase<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename Sim2TangentBase<_Derived>::Scalar
Sim2TangentBase<_Derived>::angle() const
{
  return coeffs().z();
}

template <typename _Derived>
typename Sim2TangentBase<_Derived>::Scalar
Sim2TangentBase<_Derived>::sigma() const
{
  return coeffs().w();
}

namespace internal {

//! @brief Generator specialization for Sim2TangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<Sim2TangentBase<Derived>>
{
  static typename Sim2TangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    using LieAlg = typename Sim2TangentBase<Derived>::LieAlg;
    using Scalar = typename Sim2TangentBase<Derived>::Scalar;

    switch (i)
    {
      case 0:
      {
        const static LieAlg E0((
          LieAlg() << Scalar(0), Scalar(0), Scalar(1),
                      Scalar(0), Scalar(0), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(0)
        ).finished());
        return E0;
      }
      case 1:
      {
        const static LieAlg E1((
          LieAlg() << Scalar(0), Scalar(0), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(1),
                      Scalar(0), Scalar(0), Scalar(0)
        ).finished());
        return E1;
      }
      case 2:
      {
        const static LieAlg E2((
          LieAlg() << Scalar(0), Scalar(-1), Scalar(0),
                      Scalar(1), Scalar( 0), Scalar(0),
                      Scalar(0), Scalar( 0), Scalar(0)
        ).finished());
        return E2;
      }
      case 3:
      {
        const static LieAlg E3((
          LieAlg() << Scalar(1), Scalar(0), Scalar(0),
                      Scalar(0), Scalar(1), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(0)
        ).finished());
        return E3;
      }
      default:
        MANIF_THROW("Index i must be in [0,3]!", invalid_argument);
        break;
    }

    return LieAlg{};
  }
};

//! @brief Random specialization for Sim2TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<Sim2TangentBase<Derived>>
{
  static void run(Sim2TangentBase<Derived>& m)
  {
    m.coeffs().setRandom();             // in [-1,1]
    m.coeffs().coeffRef(2) *= MANIF_PI; // in [-PI,PI]
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SIM2TANGENT_BASE_H_
