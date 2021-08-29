#ifndef _MANIF_MANIF_SIM3TANGENT_BASE_H_
#define _MANIF_MANIF_SIM3TANGENT_BASE_H_

#include "manif/impl/sim3/Sim3_properties.h"
#include "manif/impl/tangent_base.h"
#include "manif/impl/rxso3/RxSO3Tangent_map.h"

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the Sim3 tangent.
 * @note See Appendix D of the paper.
 */
template <typename _Derived>
struct Sim3TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = Sim3TangentBase<_Derived>;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using LinBlock = typename DataType::template FixedSegmentReturnType<3>::Type;
  using AngBlock = typename DataType::template FixedSegmentReturnType<3>::Type;
  using ScaBlock = typename DataType::template FixedSegmentReturnType<1>::Type;
  using ConstLinBlock = typename DataType::template ConstFixedSegmentReturnType<3>::Type;
  using ConstAngBlock = typename DataType::template ConstFixedSegmentReturnType<3>::Type;
  using ConstScaBlock = typename DataType::template ConstFixedSegmentReturnType<1>::Type;

  using Base::data;
  using Base::coeffs;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(Sim3TangentBase)

public:

  MANIF_TANGENT_ML_ASSIGN_OP(Sim3TangentBase)

  // Tangent common API

  /**
   * @brief Hat operator of Sim3.
   * @return An element of the Lie algebra sim3.
   */
  LieAlg hat() const;

  /**
   * @brief Get the Sim3 element.
   * @param[out] -optional- J_m_t Jacobian of the Sim3 element wrt this.
   * @return The Sim3 element.
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
   * @brief Get the right Jacobian of Sim3.
   */
  Jacobian rjac() const;

  /**
   * @brief Get the left Jacobian of Sim3.
   */
  Jacobian ljac() const;

  /**
   * @brief Get the inverse right Jacobian of Sim3.
   */
  Jacobian rjacinv() const;

  /**
   * @brief Get the inverse left Jacobian of Sim3.
   */
  Jacobian ljacinv() const;

  /**
   * @brief
   * @return
   */
  Jacobian smallAdj() const;

  // Sim3Tangent specific API

  //! @brief Get the linear part.
  LinBlock lin();
  const ConstLinBlock lin() const;

  //! @brief Get the angular part.
  AngBlock ang();
  const ConstAngBlock ang() const;

  //! @brief Get the scale part.
  ScaBlock sca();
  const ConstScaBlock sca() const;

// protected:

  const Eigen::Map<const RxSO3Tangent<Scalar>> asRxSO3() const;
  Eigen::Map<RxSO3Tangent<Scalar>> asRxSO3();

  const Eigen::Map<const SO3Tangent<Scalar>> asSO3() const;
  Eigen::Map<SO3Tangent<Scalar>> asSO3();

  template <typename _EigenDerived>
  static void fillQ(
    Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> Q,
    const Eigen::MatrixBase<_EigenDerived>& c
  );

  static void fillW(
    Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> W,
    const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>& phi,
    const Scalar sigma,
    const tl::optional<Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>> lin = {},
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 4>>> J_t_phisigma = {}
  );
  static void fillWinv(
    Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> W,
    const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>& phi,
    const Scalar sigma
  );
  static Eigen::Matrix<Scalar, 3, 3> computeW(
    const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>& phi,
    const Scalar sigma,
    const tl::optional<Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>> lin = {},
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 4>>> J_t_phisigma = {}
  );
  static Eigen::Matrix<Scalar, 3, 3> computeWinv(
    const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>& phi,
    const Scalar sigma
  );
};

template <typename _Derived>
typename Sim3TangentBase<_Derived>::LieGroup
Sim3TangentBase<_Derived>::exp(OptJacobianRef J_m_t) const
{
  if (J_m_t) {
    *J_m_t = rjac();
  }

  const Eigen::Map<const RxSO3Tangent<Scalar>> RxSO3tan = asRxSO3();

  return LieGroup(
    computeW(RxSO3tan.ang(), RxSO3tan.s())*lin(), RxSO3tan.exp()
  );
}

template <typename _Derived>
typename Sim3TangentBase<_Derived>::LieGroup
Sim3TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename Sim3TangentBase<_Derived>::LieAlg
Sim3TangentBase<_Derived>::hat() const
{
  LieAlg hat;
  hat.template topLeftCorner<3, 3>() = asRxSO3().hat();
  hat.template topRightCorner<3, 1>() = lin();
  hat.template bottomRows<1>().setZero();

  return hat;
}

template <typename _Derived>
typename Sim3TangentBase<_Derived>::Jacobian
Sim3TangentBase<_Derived>::rjac() const
{
  /// this is
  ///       rjac =  | W(-tau)     ?     ? |
  ///               | 0       rjac(SO3) 0 |
  ///               | 0           0     1 |
  ///
  /// considering vee(log(g)) = (v;w;s)

  Jacobian Jr;

  fillW(
    Jr.template topLeftCorner<3,3>(),
    -ang(),
    -coeffs()(6),
    lin(),
    Jr.template topRightCorner<3,4>()
  );

  fillQ( Jr.template block<3,3>(0,3), -coeffs() );
  // Jr.template block<3,3>(0,3) -= Jr.template block<3,3>(0,3) * coeffs()(6);

  // Jr.template topRightCorner<3,1>() = Jr.template block<3,3>(0,3) * (-lin());

  Jr.template block<3,3>(3,3) = asSO3().rjac();

  Jr.template bottomLeftCorner<4,3>().setZero();
  Jr.template block<3,1>(3,6).setZero();
  Jr.template block<1,3>(6,3).setZero();
  Jr(6, 6) = Scalar(1);

  return Jr;
}

template <typename _Derived>
typename Sim3TangentBase<_Derived>::Jacobian
Sim3TangentBase<_Derived>::ljac() const
{
  /// this is
  ///       rjac =  |      W         Q       -lin |
  ///               |      0     rjac(SO3)     0  |
  ///               |      0         0         1  |
  ///
  /// considering vee(log(g)) = (v;w;s)

  Jacobian Jl;
  fillW(Jl.template topLeftCorner<3,3>(), ang(), coeffs()(6),
   {}, Jl.template bottomLeftCorner<3,3>());
  Jl.template topRightCorner<3,1>() = Jl.template bottomLeftCorner<3,3>()*lin();
  //  {}, Jl.template topRightCorner<3,1>());


  fillQ( Jl.template block<3,3>(0,3), coeffs() );
  // Jl.template block<3,3>(0,3) = computeW(ang(), coeffs()(6));
  // Jl.template topRightCorner<3,1>() = lin();

  Jl.template block<3,3>(3,3) = asSO3().rjac();

  Jl.template bottomLeftCorner<4,3>().setZero();
  Jl.template block<3,1>(3,6).setZero();
  Jl.template block<1,3>(6,3).setZero();
  Jl(6, 6) = Scalar(1);

  return Jl;
}

template <typename _Derived>
typename Sim3TangentBase<_Derived>::Jacobian
Sim3TangentBase<_Derived>::rjacinv() const
{
  return rjac().inverse();

  Jacobian Jr;
  Jr.template block<3,3>(3,3) = asSO3().rjacinv();
  fillW(
    Jr.template block<3,3>(0,3), // tmp W
    -ang(),
    -coeffs()(6),
   {},
   Jr.template bottomLeftCorner<3,3>()
  );
  Jr.template topLeftCorner<3,3>().noalias() =
    -Jr.template block<3,3>(3,3) *
     Jr.template block<3,3>(0,3) *
     Jr.template block<3,3>(3,3);
  Jr.template topRightCorner<3,1>() = Jr.template bottomLeftCorner<3,3>()*lin();
  // Jr.template topRightCorner<3,1>() = Jr.template bottomLeftCorner<3,3>()*(Jr.template topLeftCorner<3,3>()*lin());
  //  {}, Jr.template topRightCorner<3,1>());

  Jr.template block<3,3>(0,3).setZero();

  Jr.template bottomLeftCorner<4,3>().setZero();
  Jr.template block<3,1>(3,6).setZero();
  Jr.template block<1,3>(6,3).setZero();
  Jr(6, 6) = Scalar(1);

  return Jr;
}

template <typename _Derived>
typename Sim3TangentBase<_Derived>::Jacobian
Sim3TangentBase<_Derived>::ljacinv() const
{
  return ljac().inverse();
  // Jacobian Jl_inv;
  // fillQ( Jl_inv.template bottomLeftCorner<3,3>(), coeffs() ); // serves as temporary Q
  // Jl_inv.template topLeftCorner<3,3>() = asSO3().ljacinv();
  // Jl_inv.template bottomRightCorner<3,3>() = Jl_inv.template topLeftCorner<3,3>();
  // Jl_inv.template topRightCorner<3,3>().noalias() =
  //     -Jl_inv.template topLeftCorner<3,3>()    *
  //      Jl_inv.template bottomLeftCorner<3,3>() *
  //      Jl_inv.template topLeftCorner<3,3>();
  // Jl_inv.template bottomLeftCorner<3,3>().setZero();

  // return Jl_inv;
}

template <typename _Derived>
template <typename _EigenDerived>
void Sim3TangentBase<_Derived>::fillQ(
  Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> Q,
  const Eigen::MatrixBase<_EigenDerived>& c
) {
    using std::cos;
    using std::sin;
    using std::sqrt;

    const Scalar theta_sq = c.template segment<3>(3).squaredNorm();

    Scalar A(0.5), B, C, D;

    // Small angle approximation
    if (theta_sq <= Constants<Scalar>::eps)
    {
      B =  Scalar(1./6.)  + Scalar(1./120.) * theta_sq;
      C = -Scalar(1./24.) + Scalar(1./720.) * theta_sq;
      D = -Scalar(1./60.);
    }
    else
    {
      const Scalar theta     = sqrt(theta_sq);
      const Scalar sin_theta = sin(theta);
      const Scalar cos_theta = cos(theta);

      B = (theta - sin_theta) / (theta_sq*theta);
      C = (Scalar(1) - theta_sq/Scalar(2) - cos_theta) / (theta_sq*theta_sq);
      D = (C - Scalar(3)*(theta-sin_theta-theta_sq*theta/Scalar(6)) / (theta_sq*theta_sq*theta));

      // http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17_identities.pdf
  //    C = (theta_sq+Scalar(2)*cos_theta-Scalar(2)) / (Scalar(2)*theta_sq*theta_sq);
  //    D = (Scalar(2)*theta - Scalar(3)*sin_theta + theta*cos_theta) / (Scalar(2)*theta_sq*theta_sq*theta);
    }

    // const Scalar scale = c(6);

    /// @note Barfoot14tro Eq. 102
    const Eigen::Matrix<Scalar, 3, 3> V   = skew(c.template head<3>());
    const Eigen::Matrix<Scalar, 3, 3> W   = skew(c.template segment<3>(3));
    const Eigen::Matrix<Scalar, 3, 3> VW  = V * W;
    const Eigen::Matrix<Scalar, 3, 3> WV  = VW.transpose();       // Note on this change wrt. Barfoot: it happens that V*W = (W*V).transpose() !!!
    const Eigen::Matrix<Scalar, 3, 3> WVW = WV * W;
    const Eigen::Matrix<Scalar, 3, 3> VWW = VW * W;

    Q.noalias() =
      + A * V
      + B * (WV + VW + WVW)
      - C * (VWW - VWW.transpose() - Scalar(3) * WVW)           // Note on this change wrt. Barfoot: it happens that V*W*W = -(W*W*V).transpose() !!!
      - D * WVW * W;                                            // Note on this change wrt. Barfoot: it happens that W*V*W*W = W*W*V*W !!!
}


template <typename _Derived>
typename Sim3TangentBase<_Derived>::Jacobian
Sim3TangentBase<_Derived>::smallAdj() const
{
  /// this is
  ///       ad(g) = |  Omega+I.s   V   -v |
  ///               |      0     Omega  0 |
  ///               |      0       0    0 |
  ///
  /// with Omega = [w]x & V = [v]x
  ///
  /// considering vee(log(g)) = (v;w;s)

  Jacobian smallAdj;
  smallAdj.template block<3,3>(3,3) = skew(ang());

  smallAdj.template topLeftCorner<3,3>() =
    smallAdj.template block<3,3>(3,3) +
    coeffs()(6) * Eigen::Matrix<Scalar,3,3>::Identity();

  smallAdj.template block<3,3>(0,3) = skew(lin());
  smallAdj.template topRightCorner<3,1>() = -lin();

  smallAdj.template bottomLeftCorner<4,3>().setZero();
  smallAdj.template bottomRightCorner<4,1>().setZero();
  smallAdj.template block<1,3>(6,3).setZero();

  return smallAdj;
}

// Sim3Tangent specific API

template <typename _Derived>
typename Sim3TangentBase<_Derived>::LinBlock
Sim3TangentBase<_Derived>::lin()
{
  return coeffs().template head<3>();
}

template <typename _Derived>
const typename Sim3TangentBase<_Derived>::ConstLinBlock
Sim3TangentBase<_Derived>::lin() const
{
  return coeffs().template head<3>();
}

template <typename _Derived>
typename Sim3TangentBase<_Derived>::AngBlock
Sim3TangentBase<_Derived>::ang()
{
  return coeffs().template segment<3>(3);
}

template <typename _Derived>
const typename Sim3TangentBase<_Derived>::ConstAngBlock
Sim3TangentBase<_Derived>::ang() const
{
  return coeffs().template segment<3>(3);
}

template <typename _Derived>
typename Sim3TangentBase<_Derived>::ScaBlock
Sim3TangentBase<_Derived>::sca()
{
  return coeffs().template tail<1>();
}

template <typename _Derived>
const typename Sim3TangentBase<_Derived>::ConstScaBlock
Sim3TangentBase<_Derived>::sca() const
{
  return coeffs().template tail<1>();
}

template <typename _Derived>
const Eigen::Map<const RxSO3Tangent<typename Sim3TangentBase<_Derived>::Scalar>>
Sim3TangentBase<_Derived>::asRxSO3() const
{
  return Eigen::Map<const RxSO3Tangent<Scalar>>(coeffs().data()+3);
}

template <typename _Derived>
Eigen::Map<RxSO3Tangent<typename Sim3TangentBase<_Derived>::Scalar>>
Sim3TangentBase<_Derived>::asRxSO3()
{
  return Eigen::Map<RxSO3Tangent<Scalar>>(coeffs().data()+3);
}

template <typename _Derived>
const Eigen::Map<const SO3Tangent<typename Sim3TangentBase<_Derived>::Scalar>>
Sim3TangentBase<_Derived>::asSO3() const
{
  return Eigen::Map<const SO3Tangent<Scalar>>(coeffs().data()+3);
}

template <typename _Derived>
Eigen::Map<SO3Tangent<typename Sim3TangentBase<_Derived>::Scalar>>
Sim3TangentBase<_Derived>::asSO3()
{
  return Eigen::Map<SO3Tangent<Scalar>>(coeffs().data()+3);
}

template <typename _Derived>
void Sim3TangentBase<_Derived>::fillW(
  Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> W,
  const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>& phi,
  const Scalar sigma,
  const tl::optional<Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>> lin,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 4>>> J_t_phisigma
) {
  using SO3LieAlg = typename SO3Tangent<Scalar>::LieAlg;
  using std::abs;
  using std::cos;
  using std::sin;
  using std::exp;
  using std::sqrt;

  assert_vector_dim(phi, 3);

  const SO3LieAlg I = SO3LieAlg::Identity();
  const Scalar half(0.5);
  const Scalar one(1);
  const Scalar two(2);
  const Scalar three(3);
  const Scalar six(6);
  const Scalar twenty_four(24);

  const Scalar theta_sq = phi.squaredNorm();
  // sqrt(Jet(0)) results in NaNs derivatives
  // thus we use an iterative approximation when close to zero.
  const Scalar theta = theta_sq > Constants<Scalar>::eps?
    sqrt(theta_sq) : internal::csqrt(theta_sq);
  const Scalar scale = exp(sigma);

  const SO3LieAlg Phi = Eigen::Map<const SO3Tangent<Scalar>>(phi.data()).hat();
  const SO3LieAlg Phi2 = Phi * Phi;

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

      if (J_t_phisigma) {

        MANIF_CHECK(lin);

        const Scalar sigma_sq = sigma * sigma;

        const Scalar b_one = b - Scalar(1);
        const Scalar c_sq = c * c;
        const Scalar b_theta = theta * b;
        const Scalar c_theta = theta * c;
        const Scalar a_sigma = sigma * a;
        const Scalar two_sigma = sigma * Scalar(2);

        const Scalar A_dtheta =
          (two * (theta * (b - one) - (a * sigma))) / (c*c) +
          ((b * sigma) - b + (a * theta) + one) / (c * theta) +
          (theta * (b - one) - (a * sigma)) / (theta_sq * c);

        const Scalar B_dtheta = (
            ((two * theta) * ((b * sigma) - sigma + (a * theta))) / (c*c) -
            ((a + (b * theta) - (a * sigma))) / c
          ) / theta_sq - (
            two * ((scale - one) / sigma - ((b * sigma) - sigma + (a * theta)) / c)
          ) / (theta * c);

        // const Scalar B_dtheta =
        //   (two * (sigma * (b - one) + (theta * a))) / (theta * c_sq) +
        //   (sigma * a + a + theta * b) / (theta_sq * c) +
        //   (two * (sigma_sq * (b - one) - c * (scale - one))) / (theta_sq * theta * c);

        const Eigen::Matrix<Scalar, 3, 1> G_theta_phi = phi / theta;

        for (int i=0; i<3; ++i) {
          const SO3LieAlg J_Phi_phi = skew(Eigen::Matrix<Scalar, 3, 1>::Unit(i));
          const SO3LieAlg J_Phi2_phi = J_Phi_phi * Phi + Phi * J_Phi_phi;

          J_t_phisigma->col(i) = (
            A_dtheta * G_theta_phi(i) * Phi + A * J_Phi_phi +
            B_dtheta * G_theta_phi(i) * Phi2 + B * J_Phi2_phi
          ) * (*lin);
        }

        const Scalar A_dsigma =
          (a - b_theta + a_sigma) / c_theta -
          (two_sigma * (theta - b_theta + a_sigma)) / (theta * c_sq);

        const Scalar B_dsigma = -(
          (sigma * b + theta * a + b_one) / c + (scale - one) / sigma_sq -
          (two_sigma * (sigma * b_one + theta * a)) / c_sq - scale / sigma
        ) / theta_sq;

        // const Scalar B_dsigma =
        //   (two * sigma_sq * b_one + two * sigma * a * theta) / (theta_sq * c_sq) -
        //   (b*(sigma - scale) - one) / (theta_sq * c) -
        //   (sigma * scale - scale - one) / (sigma_sq * theta_sq);

        const Scalar C_dsigma = (one + scale * (sigma - one)) / sigma_sq;

        J_t_phisigma->col(3) = (A_dsigma * Phi + B_dsigma * Phi2 + C_dsigma * I) * (*lin);
      }
    }
  }

  W = A * Phi + B * Phi2 + C * I;

  // if (J_W_sigma) {
  //   J_W_sigma->setZero();
  //   if (abs(sigma) < Constants<Scalar>::eps) {
  //     // C = one;
  //     // C_dsigma = half;
  //     if (abs(theta) < Constants<Scalar>::eps) {
  //       // A = half;
  //       // B = Scalar(1. / 6.);
  //       // // A_dtheta = A_dsigma = Scalar(0);
  //       // // B_dtheta = B_dsigma = Scalar(0);

  //       J_W_sigma->setZero();
  //       // *J_W_sigma << dA, dB, half;
  //     } else {
  //       // A = (one - cos_theta) / theta_sq;
  //       // B = (theta - sin_theta) / theta_c;
  //       // A_dtheta = (theta * sin_theta + two * cos_theta - two) / theta_c;
  //       // B_dtheta = -(two * theta - three * sin_theta + theta * cos_theta) /
  //       //           (theta_c * theta);
  //       // // A_dsigma = (sin_theta - theta * cos_theta) / theta_c;
  //       // // B_dsigma =
  //       // //     (half - (cos_theta + theta * sin_theta - one) / theta_sq) / theta_sq;

  //       // const Scalar theta_sq = theta * theta;
  //       // *J_W_sigma << (sin(theta) - theta * cos(theta)) / (theta_sq * theta),
  //       //               (Scalar(0.5) - (cos(theta) + theta * sin(theta) - Scalar(1)) / theta_sq) / theta_sq,
  //       //               Scalar(0);
  //     }
  //   } else {
  //     C = (scale - one) / sigma;
  //     const Scalar sigma_sq = sigma * sigma;
  //     // const Scalar sigma_c = sigma * sigma_sq;
  //     const Scalar C_dsigma = (one + scale * (sigma - one)) / sigma_sq;
  //     // const Scalar C_dsigma = (one - scale * (sigma - one)) / sigma_sq;

  //     if (abs(theta) < Constants<Scalar>::eps) {
  //       // A = ((sigma - one) * scale + one) / sigma_sq;
  //       // B = (scale * half * sigma_sq + scale - one - sigma * scale) / sigma_c;
  //       // // A_dsigma = (scale * (sigma_sq - two * sigma + two) - two) / sigma_c;
  //       // // B_dsigma = (scale * (half * sigma_c - (one + half) * sigma_sq +
  //       // //                     three * sigma - three) +
  //       // //             three) /
  //       // //           (sigma_c * sigma);
  //       // A_dtheta = B_dtheta = zero;

  //       // *J_W_sigma << (scale * (sigma_sq - Scalar(2) * sigma + Scalar(2)) - Scalar(2)) / sigma_c,
  //       //               (scale * (Scalar(0.5) * sigma_c - Scalar(1.5) * sigma_sq + Scalar(3) * sigma - Scalar(3)) + Scalar(3)) / (sigma_c * sigma),
  //       //               C_dsigma;
  //     } else {
  //       const Scalar theta_sq = theta * theta;

  //       const Scalar sin_theta = sin(theta);
  //       const Scalar cos_theta = cos(theta);
  //       const Scalar a = scale * sin_theta;
  //       const Scalar b = scale * cos_theta;
  //       const Scalar b_one = b - Scalar(1);
  //       // const Scalar theta_b_one = theta * b_one;
  //       const Scalar c = theta_sq + sigma_sq;
  //       const Scalar c_sq = c * c;
  //       // const Scalar theta_sq_c = theta_sq * c;
  //       const Scalar a_theta = theta * a;
  //       const Scalar b_theta = theta * b;
  //       const Scalar c_theta = theta * c;
  //       const Scalar a_sigma = sigma * a;
  //       const Scalar b_sigma = sigma * b;
  //       const Scalar two_sigma = sigma * Scalar(2);
  //       // const Scalar two_theta = theta * Scalar(2);
  //       const Scalar sigma_b_one = sigma * b_one;

  //       // A = (a_sigma - theta_b_one) / c_theta;
  //       // A_dtheta = (two * (theta_b_one - a_sigma)) / c_sq +
  //       //           (b_sigma - b + a_theta + one) / c_theta +
  //       //           (theta_b_one - a_sigma) / theta_sq_c;
  //       // // A_dsigma = (a - b_theta + a_sigma) / c_theta -
  //       // //           (two_sigma * (theta - b_theta + a_sigma)) / (theta * c_sq);

  //       // B = (C - (sigma_b_one + a_theta) / (c)) * one / (theta_sq);
  //       // B_dtheta =
  //       //     ((two_theta * (b_sigma - sigma + a_theta)) / c_sq -
  //       //     ((a + b_theta - a_sigma)) / c) /
  //       //         theta_sq -
  //       //     (two * ((scale - one) / sigma - (b_sigma - sigma + a_theta) / c)) /
  //       //         theta_c;
  //       // // B_dsigma =
  //       // //     -((b_sigma + a_theta + b_one) / c + (scale - one) / sigma_sq -
  //       // //       (two_sigma * (sigma_b_one + a_theta)) / c_sq - scale / sigma) /
  //       // //     theta_sq;

  //       const Scalar A_dsigma = (a - b_theta + a_sigma) / c_theta - (two_sigma * (theta - b_theta + a_sigma)) / (theta * c_sq);
  //       const Scalar B_dsigma = -((b_sigma + a_theta + b_one) / c + (scale - Scalar(1)) / sigma_sq - (two_sigma * (sigma_b_one + a_theta)) / c_sq - scale / sigma) / theta_sq;

  //       // const Scalar A_dsigma = (
  //       //   -two*sigma*(sigma*scale*sin_theta - theta*scale*cos_theta + theta) +
  //       //   scale*c*(sigma*sin_theta - theta*cos_theta + sin_theta)
  //       // ) / (theta * c * c);

  //       // const Scalar B_dsigma = (
  //       //   two*sigma_sq*sigma_sq*(scale*cos_theta-one) -
  //       //   scale*sin_theta*sigma_sq*theta*c*c*(sigma_sq+two*sigma+theta_sq) +
  //       //   sigma_sq*c*(-sigma*scale*cos_theta-scale*cos_theta+one) +
  //       //   sigma*c*c*scale +
  //       //   (one-scale)*c*c
  //       // ) / (sigma_sq*theta_sq*c*c);

  //       // *J_W_sigma << (a - b_theta + a_sigma) / c_theta - (two_sigma * (theta - b_theta + a_sigma)) / (theta * c_sq),
  //       //               -((b_sigma + a_theta + b_one) / c + (scale - Scalar(1)) / sigma_sq - (two_sigma * (sigma_b_one + a_theta)) / c_sq - scale / sigma) / theta_sq,
  //       //               C_dsigma;

  //       // std::cout << "A_dsigma " << A_dsigma << std::endl;
  //       // std::cout << "B_dsigma " << B_dsigma << std::endl;
  //       // std::cout << "C_dsigma " << C_dsigma << std::endl;

  //       J_W_sigma->noalias() = A_dsigma * Phi + B_dsigma * Phi2 + C_dsigma * I;
  //     }
  //   }
  // }
}

template <typename _Derived>
void Sim3TangentBase<_Derived>::fillWinv(
  Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> W,
  const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>& phi,
  const Scalar sigma
) {
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  // using SO3LieAlg = typename SO3Tangent<_Derived>::LieAlg;
  using std::abs;
  using std::cos;
  using std::sin;
  using std::exp;

  assert_vector_dim(phi, 3);

  const Matrix3 I = Matrix3::Identity();
  const Scalar one(1);
  const Scalar half(0.5);

  const Scalar theta = phi.norm();

  // const SO3LieAlg Phi = Eigen::Map<const SO3Tangent<Scalar>>(phi.data()).hat();
  // const SO3LieAlg Phi2 = Phi * Phi;
  const Matrix3 Phi = Eigen::Map<const SO3Tangent<Scalar>>(phi.data()).hat();
  const Matrix3 Phi2 = Phi * Phi;
  const Scalar scale = exp(sigma);

  Scalar A, B, C;
  if (abs(sigma) < Constants<Scalar>::eps) {
    C = one;
    if (abs(theta) < Constants<Scalar>::eps) {
      A = half;
      B = Scalar(1. / 6.);
    } else {
      const Scalar theta_sq = theta * theta;
      A = (one - cos(theta)) / theta_sq;
      B = (theta - sin(theta)) / (theta_sq * theta);
    }
  } else {
    C = (scale - one) / sigma;
    if (abs(theta) < Constants<Scalar>::eps) {
      Scalar sigma_sq = sigma * sigma;
      A = ((sigma - one) * scale + one) / sigma_sq;
      B = (scale * half * sigma_sq + scale - one - sigma * scale) /
          (sigma_sq * sigma);
    } else {
      Scalar theta_sq = theta * theta;
      Scalar a = scale * sin(theta);
      Scalar b = scale * cos(theta);
      Scalar c = theta_sq + sigma * sigma;
      A = (a * sigma + (one - b) * theta) / (theta * c);
      B = (C - ((b - one) * sigma + a * theta) / (c)) * one / (theta_sq);
    }
  }
  W = A * Phi + B * Phi2 + C * I;
}

template <typename _Derived>
Eigen::Matrix<typename Sim3TangentBase<_Derived>::Scalar, 3, 3>
Sim3TangentBase<_Derived>::computeW(
  const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>& phi,
  const Scalar sigma,
  const tl::optional<Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>> lin,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 4>>> J_t_phisigma
) {
  Eigen::Matrix<Scalar, 3, 3> W;
  fillW(W, phi, sigma, lin, J_t_phisigma);
  return W;
}

template <typename _Derived>
Eigen::Matrix<typename Sim3TangentBase<_Derived>::Scalar, 3, 3>
Sim3TangentBase<_Derived>::computeWinv(
  const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>>& phi, const Scalar sigma
) {
  Eigen::Matrix<Scalar, 3, 3> W;
  fillWinv(W, phi, sigma);
  return W;
}

namespace internal {

//! @brief Generator specialization for Sim3TangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<Sim3TangentBase<Derived>>
{
  static typename Sim3TangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    using LieAlg = typename Sim3TangentBase<Derived>::LieAlg;
    using Scalar = typename Sim3TangentBase<Derived>::Scalar;

    switch (i)
    {
      case 0:
      {
        static const LieAlg E0((
          LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(1),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(0)
        ).finished());
        return E0;
      }
      case 1:
      {
        static const LieAlg E1((
          LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(1),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(0)
        ).finished());
        return E1;
      }
      case 2:
      {
        static const LieAlg E2((
          LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(1),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(0)
        ).finished());
        return E2;
      }
      case 3:
      {
        static const LieAlg E3((
          LieAlg() << Scalar(0), Scalar(0), Scalar( 0), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(-1), Scalar(0),
                      Scalar(0), Scalar(1), Scalar( 0), Scalar(0),
                      Scalar(0), Scalar(0), Scalar( 0), Scalar(0)
        ).finished());
        return E3;
      }
      case 4:
      {
        static const LieAlg E4((
          LieAlg() << Scalar( 0), Scalar(0), Scalar(1), Scalar(0),
                      Scalar( 0), Scalar(0), Scalar(0), Scalar(0),
                      Scalar(-1), Scalar(0), Scalar(0), Scalar(0),
                      Scalar( 0), Scalar(0), Scalar(0), Scalar(0)
        ).finished());
        return E4;
      }
      case 5:
      {
        static const LieAlg E5((
          LieAlg() << Scalar(0), Scalar(-1), Scalar(0), Scalar(0),
                      Scalar(1), Scalar( 0), Scalar(0), Scalar(0),
                      Scalar(0), Scalar( 0), Scalar(0), Scalar(0),
                      Scalar(0), Scalar( 0), Scalar(0), Scalar(0)
        ).finished());
        return E5;
      }
      case 6:
      {
        static const LieAlg E6((
          LieAlg() << Scalar(1), Scalar(0), Scalar(0), Scalar(0),
                      Scalar(0), Scalar(1), Scalar(0), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(1), Scalar(0),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(0)
        ).finished());
        return E6;
      }
      default:
        MANIF_THROW("Index i must be in [0,6]!", invalid_argument);
        break;
    }

    return LieAlg{};
  }
};

//! @brief Random specialization for Sim3TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<Sim3TangentBase<Derived>>
{
  static void run(Sim3TangentBase<Derived>& m)
  {
    using Scalar = typename Derived::Scalar;

    m.coeffs().template head<3>().setRandom();
    // In ball of radius PI
    m.coeffs().template segment<3>(3) =
      randPointInBall(MANIF_PI).template cast<Scalar>();
    m.coeffs()(6) = Eigen::internal::random<Scalar>();
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SIM3TANGENT_BASE_H_
