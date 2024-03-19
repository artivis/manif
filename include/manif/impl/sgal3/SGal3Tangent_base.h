#ifndef _MANIF_MANIF_SGAL3TANGENT_BASE_H_
#define _MANIF_MANIF_SGAL3TANGENT_BASE_H_

#include "manif/impl/sgal3/SGal3_properties.h"
#include "manif/impl/tangent_base.h"
#include "manif/impl/so3/SO3Tangent_map.h"
#include "manif/impl/se3/SE3Tangent.h"

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the SGal3 tangent.
 */
template <typename _Derived>
struct SGal3TangentBase : TangentBase<_Derived> {
private:

  using Base = TangentBase<_Derived>;
  using Type = SGal3TangentBase<_Derived>;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  using LinBlock = typename DataType::template FixedSegmentReturnType<3>::Type;
  using AngBlock = typename DataType::template FixedSegmentReturnType<3>::Type;
  using ConstLinBlock = typename DataType::template ConstFixedSegmentReturnType<3>::Type;
  using ConstAngBlock = typename DataType::template ConstFixedSegmentReturnType<3>::Type;

  using Base::data;
  using Base::coeffs;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SGal3TangentBase)

public:

  MANIF_TANGENT_ML_ASSIGN_OP(SGal3TangentBase)

  // Tangent common API

  /**
   * @brief Hat operator of SGal3.
   * @return An element of the Lie algebra se_2_3.
   * @note See Eq. (169).
   */
  LieAlg hat() const;

  /**
   * @brief Get the SGal3 element.
   * @param[out] -optional- J_m_t Jacobian of the SGal3 element wrt this.
   * @return The SGal3 element.
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
   * @brief Get the right Jacobian of SGal3.
   */
  Jacobian rjac() const;

  /**
   * @brief Get the left Jacobian of SGal3.
   */
  Jacobian ljac() const;

  /**
   * @brief Get the small adjoint matrix ad() of SGal3
   * that maps isomorphic tangent vectors of SGal3
   * @return
   */
  Jacobian smallAdj() const;

  // SGal3Tangent specific API

  //! @brief Get the linear velocity part.
  LinBlock lin();
  const ConstLinBlock lin() const;

  //! @brief Get the angular part.
  AngBlock ang();
  const ConstAngBlock ang() const;

  //! @brief Get the linear acceleration part
  LinBlock lin2();
  const ConstLinBlock lin2() const;

  Scalar t() const;

public: /// @todo make protected

  const Eigen::Map<const SO3Tangent<Scalar>> asSO3() const {
    return Eigen::Map<const SO3Tangent<Scalar>>(coeffs().data()+6);
  }

  Eigen::Map<SO3Tangent<Scalar>> asSO3() {
    return Eigen::Map<SO3Tangent<Scalar>>(coeffs().data()+6);
  }

  static void fillE(
    Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> E,
    const Eigen::Map<const SO3Tangent<Scalar>>& so3
  );

  // static void fillL(
  //   Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> E,
  //   const Eigen::Map<const SO3Tangent<Scalar>>& so3
  // );

  // static void fillS(
    // Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> S,
    // const Eigen::Map<const SO3Tangent<Scalar>>& so3,
    // linearVel,
    // t
  // );
};

template <typename _Derived>
typename SGal3TangentBase<_Derived>::LieGroup
SGal3TangentBase<_Derived>::exp(OptJacobianRef J_m_t) const {
  if (J_m_t) {
    *J_m_t = rjac();
  }

  const Eigen::Map<const SO3Tangent<Scalar>> so3 = asSO3();
  const typename SO3<Scalar>::Jacobian so3_ljac = so3.ljac();

  Eigen::Matrix<Scalar, 3, 3> E;
  fillE(E, so3);

  return LieGroup(
    so3_ljac * lin() + (E * (t() * lin2())),
    so3.exp(),
    so3_ljac * lin2(),
    t()
  );
}

template <typename _Derived>
typename SGal3TangentBase<_Derived>::LieGroup
SGal3TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const {
  return exp(J_m_t);
}

template <typename _Derived>
typename SGal3TangentBase<_Derived>::LieAlg
SGal3TangentBase<_Derived>::hat() const {
  LieAlg sgal3;

  sgal3.template topLeftCorner<3, 3>() = skew(ang());
  sgal3.template block<3, 1>(0, 3) = lin2();
  sgal3.template topRightCorner<3, 1>() = lin();
  sgal3(3, 4) = t();

  sgal3.template bottomLeftCorner<2, 4>().setZero();
  sgal3(4, 4) = Scalar(0);

  return sgal3;
}

template <typename _Derived>
typename SGal3TangentBase<_Derived>::Jacobian
SGal3TangentBase<_Derived>::rjac() const {

  return (-*this).ljac();

  // Those were verified against auto diff

  // Jacobian Jr = Jacobian::Zero();
  // Jr.template topLeftCorner<3, 3>() = asSO3().rjac();
  // // Jr.template block<3, 3>(0, 3) = ??;
  // // Jr.template block<3, 3>(0, 6) = ??;

  // // Jr.template block<3, 1>(0, 9) = ??;

  // Jr.template block<3, 3>(3, 3) = Jr.template topLeftCorner<3,3>();
  // SE3Tangent<Scalar>::fillQ(
  //   Jr.template block<3, 3>(3, 6), -coeffs().template segment<6>(3)
  // );

  // Jr.template block<3, 3>(6, 6) = Jr.template topLeftCorner<3,3>();

  // Jr(9, 9) = Scalar(1);

  // Jr.template bottomLeftCorner<7, 3>().setZero();
  // Jr.template block<4, 3>(6, 3).setZero();
  // Jr.template block<1, 3>(9, 6).setZero();
  // Jr.template block<6, 1>(3, 9).setZero();

  // return Jr;
}

template <typename _Derived>
typename SGal3TangentBase<_Derived>::Jacobian
SGal3TangentBase<_Derived>::ljac() const {
  using Diag = typename Eigen::DiagonalMatrix<Scalar, 3>;
  auto I33 = [](const Scalar d){ return Diag(d, d, d).toDenseMatrix(); };
  using Ref33 = Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>;
  using ConstRef33 = const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 3>>;

  using std::sqrt;
  using std::cos;
  using std::sin;

  /** Structure of the left Jacobian according to J. Kelly
   * 
   * Jl = [ D -L*t  N   E*nu
   *        0   D   M   0
   *        0   0   D   0
   *        0   0   0   1 ]
   * 
   * with N = N1 - N2.
   * 
   * Tangent space is tau = [rho ; nu ; theta ; t] in R^10
   * 
   * Matrix blocks D, E, L, M, N, N1, N2 are referred to in the comments and correspond to eqs. in Kelly's paper:
   * 
   *    D:  (18) = Jl_SO3(theta)
   *    E:  (19)
   *    L:  (32)
   *    M:  (33) = Q(nu,theta)
   *    N:  (34) = N1 - N2
   *    N1: (35) = Q(rho,theta)
   *    N2: (36)
   */



  Jacobian Jl;

  const Eigen::Map<const SO3Tangent<Scalar>> so3 = asSO3();               // theta vector

  // Blocks D
  Eigen::Matrix<Scalar,3,3> D = so3.ljac(); 
  Jl.template topLeftCorner<3, 3>() = D;                                  // Block D
  Jl.template block<3, 3>(3, 3)     = D;                                  // Block D
  Jl.template block<3, 3>(6, 6)     = D;                                  // Block D

  // Block E
  Eigen::Matrix<Scalar,3,3> E;
  fillE(E, so3);                                                          // Block E
  Jl.template block<3,1>(0,9) = E * lin2();                               // Block E * nu

  // Skew matrix W = theta^
  Eigen::Matrix<Scalar,3,3> W = so3.hat();
  // Skew matrix V = nu^
  Eigen::Matrix<Scalar,3,3> V = skew(lin2());

  // Angles and trigonometry
  const Scalar theta_sq = so3.coeffs().squaredNorm();
  const Scalar theta    = sqrt(theta_sq); // rotation angle
  const Scalar theta_cu = theta * theta_sq;

  const Scalar sin_t = sin(theta);
  const Scalar cos_t = cos(theta);

  // Block L
  Scalar cA, cB;
  // small angle approx.
  if (theta_cu > Constants<Scalar>::eps) {
    cA = (sin_t - theta * cos_t) / theta_cu;
    cB = (theta_sq + Scalar(2) * (Scalar(1) - theta * sin_t - cos_t)) / (Scalar(2) * theta_sq * theta_sq);
  } else {
    cA = Scalar(1./3.)  - Scalar(1./30.) * theta_sq;
    cB = Scalar(1./8.);
  }
  Jl.template block<3, 3>(0, 3).noalias() = -t() * (                      // Block - L * t
    I33(Scalar(0.5)) + cA * W + cB * W * W                                // Block L
  );

  // Block M = Q(nu,theta)
  SE3Tangent<Scalar>::fillQ(Jl.template block<3, 3>(3, 6), coeffs().template segment<6>(3)); // Block M = Q(nu,theta)


  // Block N1, part of N. N1 = Q(rho,theta)
  Eigen::Matrix<Scalar, 6, 1> rho_theta;
  rho_theta << lin(), ang();
  SE3Tangent<Scalar>::fillQ(Jl.template block<3, 3>(0, 6), rho_theta);    // block N1 = Q(rho,theta)

  // Block N2, part of N
  Scalar cC, cD, cE, cF;
  if (theta_cu > Constants<Scalar>::eps) {
    cA = (Scalar(2) - theta * sin_t - Scalar(2) * cos_t) / theta_cu / theta;
    cB = (theta_cu + Scalar(6) * theta + Scalar(6) * theta * cos_t - Scalar(12) * sin_t) / (Scalar(6) * theta_cu * theta_sq);
    cC = (Scalar(12) * sin_t - theta_cu - Scalar(3) * theta_sq * sin_t - Scalar(12) * theta * cos_t) / (Scalar(6) * theta_cu * theta_sq);
    cD = (Scalar(4) + theta_sq * (Scalar(1) + cos_t) - Scalar(4) * (theta * sin_t + cos_t) ) / (Scalar(2) * theta_cu * theta_cu);
    cE = (theta_sq + Scalar(2) * (cos_t - Scalar(1))) / (Scalar(2) * theta_cu * theta);
    cF = (theta_cu + Scalar(6) * (sin_t - theta)) / (Scalar(6) * theta_cu * theta_sq);
  }else{
    cA = Scalar(1./12.);
    cB = Scalar(1./24.);
    cC = Scalar(1./10.);
    cD = Scalar(1./240.);
    cE = Scalar(1./24.);
    cF = Scalar(1./120.);
  }

  Eigen::Matrix<Scalar,3,3> N2 = t() / Scalar(6) * V                      // Block N2, part of N
    + (cA * W + cB * W * W) * (t() * V)
    // + cC * t() * (W * V * W)
    + cC * (W * V * (t() * W))
    // + cD * t() * (W * W * V * W)
    + cD * (W * W * V * (t() * W))
    + t() * V * (cE * W + cF * W * W)
    ;

  Jl.template block<3, 3>(0, 6) -= N2;                                    // Block N = N1 - N2

  // Block 1
  Jl(9, 9) = Scalar(1);

  // Blocks of zeros 
  Jl.template bottomLeftCorner<7, 3>().setZero();
  Jl.template block<4, 3>(6, 3).setZero();
  Jl.template block<1, 3>(9, 6).setZero();
  Jl.template block<6, 1>(3, 9).setZero();

  return Jl;
}

template <typename _Derived>
typename SGal3TangentBase<_Derived>::Jacobian
SGal3TangentBase<_Derived>::smallAdj() const {
  Jacobian smallAdj;

  smallAdj.template topLeftCorner<3,3>() = skew(ang());
  smallAdj.template block<3, 3>(0, 3) = -t() * Eigen::Matrix3d::Identity();
  smallAdj.template block<3, 3>(0, 6) = skew(lin());
  smallAdj.template block<3, 1>(0, 9) = lin2();

  smallAdj.template block<3, 3>(3, 3) = smallAdj.template topLeftCorner<3,3>();
  smallAdj.template block<3, 3>(3, 6) = skew(lin2());

  smallAdj.template block<3, 3>(6, 6) = smallAdj.template topLeftCorner<3,3>();

  smallAdj.template block<7, 3>(3, 0).setZero();
  smallAdj.template block<4, 3>(6, 3).setZero();

  smallAdj.template block<1, 3>(9, 6).setZero();
  smallAdj.template block<7, 1>(3, 9).setZero();

  return smallAdj;
}

// SGal3Tangent specific API

template <typename _Derived>
typename SGal3TangentBase<_Derived>::LinBlock
SGal3TangentBase<_Derived>::lin() {
  return coeffs().template head<3>();
}

template <typename _Derived>
const typename SGal3TangentBase<_Derived>::ConstLinBlock
SGal3TangentBase<_Derived>::lin() const {
  return coeffs().template head<3>();
}

template <typename _Derived>
typename SGal3TangentBase<_Derived>::LinBlock
SGal3TangentBase<_Derived>::lin2() {
  return coeffs().template segment<3>(3);
}

template <typename _Derived>
const typename SGal3TangentBase<_Derived>::ConstLinBlock
SGal3TangentBase<_Derived>::lin2() const {
  return coeffs().template segment<3>(3);
}

template <typename _Derived>
typename SGal3TangentBase<_Derived>::AngBlock
SGal3TangentBase<_Derived>::ang() {
  return coeffs().template segment<3>(6);
}

template <typename _Derived>
const typename SGal3TangentBase<_Derived>::ConstAngBlock
SGal3TangentBase<_Derived>::ang() const {
  return coeffs().template segment<3>(6);
}

template <typename _Derived>
typename SGal3TangentBase<_Derived>::Scalar
SGal3TangentBase<_Derived>::t() const {
  return coeffs()(9);
}

template <typename _Derived>
void SGal3TangentBase<_Derived>::fillE(
  Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> E,
  const Eigen::Map<const SO3Tangent<Scalar>>& so3
) {
  using I = typename Eigen::DiagonalMatrix<Scalar, 3>;

  const Scalar theta_sq = so3.coeffs().squaredNorm();

  E.noalias() = I(Scalar(0.5), Scalar(0.5), Scalar(0.5)).toDenseMatrix();

  // small angle approx.
  if (theta_sq < Constants<Scalar>::eps) {
    return;
  }

  const Scalar theta = sqrt(theta_sq); // rotation angle

  const Scalar A = (theta - sin(theta)) / theta_sq / theta;
  const Scalar B = (theta_sq + Scalar(2) * cos(theta) - Scalar(2)) / (Scalar(2) * theta_sq * theta_sq);

  const typename SO3Tangent<Scalar>::LieAlg W = so3.hat();

  E.noalias() += A * W + B * W * W;
}

namespace internal {

//! @brief Generator specialization for SGal3TangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<SGal3TangentBase<Derived>> {
  static typename SGal3TangentBase<Derived>::LieAlg
  run(const unsigned int i) {
    using LieAlg = typename SGal3TangentBase<Derived>::LieAlg;
    using Scalar = typename SGal3TangentBase<Derived>::Scalar;

    switch (i) {
      case 0: {
        static const LieAlg E0(
          (
            LieAlg() <<
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(1),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0)

          ).finished()
        );
        return E0;
      }
      case 1: {
        static const LieAlg E1(
          (
            LieAlg() <<
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(1),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0)
          ).finished()
        );
        return E1;
      }
      case 2: {
        static const LieAlg E2(
          (
            LieAlg() <<
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(1),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0)
          ).finished()
        );
        return E2;
      }
      case 3: {
        static const LieAlg E3(
          (
            LieAlg() <<
              Scalar(0), Scalar(0), Scalar(0), Scalar(1), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0)
          ).finished()
        );
        return E3;
      }
      case 4: {
        static const LieAlg E4(
          (
            LieAlg() <<
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(1), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0)
          ).finished()
        );
        return E4;
      }
      case 5: {
        static const LieAlg E5(
          (
            LieAlg() <<
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(1), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0)
          ).finished()
        );
        return E5;
      }
      case 6: {
        static const LieAlg E6(
          (
            LieAlg() <<
              Scalar(0), Scalar(0), Scalar( 0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(-1), Scalar(0), Scalar(0),
              Scalar(0), Scalar(1), Scalar( 0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar( 0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar( 0), Scalar(0), Scalar(0)
          ).finished()
        );
        return E6;
      }
      case 7: {
        static const LieAlg E7(
          (
            LieAlg() <<
              Scalar( 0), Scalar(0), Scalar(1), Scalar(0), Scalar(0),
              Scalar( 0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(-1), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar( 0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar( 0), Scalar(0), Scalar(0), Scalar(0), Scalar(0)
          ).finished()
        );
        return E7;
      }
      case 8: {
        static const LieAlg E8(
          (
            LieAlg() <<
              Scalar(0), Scalar(-1), Scalar(0), Scalar(0), Scalar(0),
              Scalar(1), Scalar( 0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar( 0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar( 0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar( 0), Scalar(0), Scalar(0), Scalar(0)
          ).finished()
        );
        return E8;
      }
      case 9: {
        static const LieAlg E9(
          (
            LieAlg() <<
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(1),
              Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0)
          ).finished()
        );
        return E9;
      }
      default:
        MANIF_THROW("Index i must be in [0,9]!", invalid_argument);
        break;
    }

    return LieAlg{};
  }
};

//! @brief Random specialization for SGal3TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<SGal3TangentBase<Derived>> {
  static void run(SGal3TangentBase<Derived>& m) {
    // in [-1,1]
    m.coeffs().setRandom();
    // In ball of radius PI
    m.coeffs().template segment<3>(6) = randPointInBall(MANIF_PI).template cast<typename Derived::Scalar>();
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SGAL3TANGENT_BASE_H_
