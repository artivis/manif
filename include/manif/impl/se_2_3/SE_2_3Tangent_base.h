#ifndef _MANIF_MANIF_SE_2_3TANGENT_BASE_H_
#define _MANIF_MANIF_SE_2_3TANGENT_BASE_H_

#include "manif/impl/se_2_3/SE_2_3_properties.h"
#include "manif/impl/tangent_base.h"
#include "manif/impl/so3/SO3Tangent_map.h"
#include "manif/impl/se3/SE3Tangent_map.h"

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the SE_2_3 tangent.
 */
template <typename _Derived>
struct SE_2_3TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SE_2_3TangentBase<_Derived>;

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

  MANIF_DEFAULT_CONSTRUCTOR(SE_2_3TangentBase)

public:

  MANIF_TANGENT_ML_ASSIGN_OP(SE_2_3TangentBase)

  // Tangent common API

  /**
   * @brief Hat operator of SE_2_3.
   * @return An element of the Lie algebra se_2_3.
   * @note See Eq. (169).
   */
  LieAlg hat() const;

  /**
   * @brief Get the SE_2_3 element.
   * @param[out] -optional- J_m_t Jacobian of the SE_2_3 element wrt this.
   * @return The SE_2_3 element.
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
   * @brief Get the right Jacobian of SE_2_3.
   */
  Jacobian rjac() const;

  /**
   * @brief Get the left Jacobian of SE_2_3.
   */
  Jacobian ljac() const;

  /**
   * @brief Get the small adjoint matrix ad() of SE_2_3
   * that maps isomorphic tangent vectors of SE_2_3
   * @return
   */
  Jacobian smallAdj() const;

  // SE_2_3Tangent specific API

  //! @brief Get the linear velocity part.
  LinBlock lin();
  const ConstLinBlock lin() const;

  //! @brief Get the angular part.
  AngBlock ang();
  const ConstAngBlock ang() const;

  //! @brief Get the linear acceleration part
  LinBlock lin2();
  const ConstLinBlock lin2() const;

public: /// @todo make protected

  const Eigen::Map<const SO3Tangent<Scalar>> asSO3() const
  {
    return Eigen::Map<const SO3Tangent<Scalar>>(coeffs().data()+3);
  }

  Eigen::Map<SO3Tangent<Scalar>> asSO3()
  {
    return Eigen::Map<SO3Tangent<Scalar>>(coeffs.data()+3);
  }

};

template <typename _Derived>
typename SE_2_3TangentBase<_Derived>::LieGroup
SE_2_3TangentBase<_Derived>::exp(OptJacobianRef J_m_t) const
{
  if (J_m_t)
  {
    *J_m_t = rjac();
  }

  const Eigen::Map<const SO3Tangent<Scalar>> so3 = asSO3();
  const typename SO3<Scalar>::Jacobian so3_ljac = so3.ljac();

  return LieGroup(so3_ljac*lin(),
                  so3.exp().quat(),
                  so3_ljac*lin2());
}

template <typename _Derived>
typename SE_2_3TangentBase<_Derived>::LieGroup
SE_2_3TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename SE_2_3TangentBase<_Derived>::LieAlg
SE_2_3TangentBase<_Derived>::hat() const
{
  return (LieAlg() <<
    Scalar(0)           , Scalar(-coeffs()(5)), Scalar( coeffs()(4)), Scalar(coeffs()(0)), Scalar(coeffs()(6)),
    Scalar( coeffs()(5)), Scalar(0)           , Scalar(-coeffs()(3)), Scalar(coeffs()(1)), Scalar(coeffs()(7)),
    Scalar(-coeffs()(4)), Scalar( coeffs()(3)), Scalar(0)           , Scalar(coeffs()(2)), Scalar(coeffs()(8)),
    Scalar(0)           , Scalar(0)           , Scalar(0)           , Scalar(0)          , Scalar(0),
    Scalar(0)           , Scalar(0)           , Scalar(0)           , Scalar(0)          , Scalar(0)            ).finished();
}

template <typename _Derived>
typename SE_2_3TangentBase<_Derived>::Jacobian
SE_2_3TangentBase<_Derived>::rjac() const
{
  Jacobian Jr;
  Jr.template block<6, 3>(3, 0).setZero();
  Jr.template block<6, 3>(0, 6).setZero();
  Jr.template topLeftCorner<3,3>() = asSO3().rjac();
  Jr.template block<3,3>(3,3) = Jr.template topLeftCorner<3,3>();
  Jr.template bottomRightCorner<3, 3>() = Jr.template topLeftCorner<3,3>();

  // fill Qv
  Eigen::Matrix<Scalar, 6, 1> vw;
  vw << Scalar(-coeffs()(0)), Scalar(-coeffs()(1)), Scalar(-coeffs()(2)),
        Scalar(-coeffs()(3)), Scalar(-coeffs()(4)), Scalar(-coeffs()(5));
  Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> Qv = Jr.template block<3,3>(0, 3);
  SE3Tangent<Scalar>::fillQ(Qv, vw);

  // fill Qa
  Eigen::Matrix<Scalar, 6, 1> aw;
  aw << Scalar(-coeffs()(6)), Scalar(-coeffs()(7)), Scalar(-coeffs()(8)),
        Scalar(-coeffs()(3)), Scalar(-coeffs()(4)), Scalar(-coeffs()(5));
  Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> Qa = Jr.template block<3,3>(6, 3);
  SE3Tangent<Scalar>::fillQ(Qa, aw);

  return Jr;
}

template <typename _Derived>
typename SE_2_3TangentBase<_Derived>::Jacobian
SE_2_3TangentBase<_Derived>::ljac() const
{
  Jacobian Jl;
  Jl.template block<6, 3>(3, 0).setZero();
  Jl.template block<6, 3>(0, 6).setZero();
  Jl.template topLeftCorner<3,3>() = asSO3().ljac();
  Jl.template block<3,3>(3,3) = Jl.template topLeftCorner<3,3>();
  Jl.template bottomRightCorner<3, 3>() = Jl.template topLeftCorner<3,3>();

  // fill Qv
  Eigen::Matrix<Scalar, 6, 1> vw;
  vw << Scalar(coeffs()(0)), Scalar(coeffs()(1)), Scalar(coeffs()(2)),
        Scalar(coeffs()(3)), Scalar(coeffs()(4)), Scalar(coeffs()(5));
  Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> Qv = Jl.template block<3,3>(0, 3);
  SE3Tangent<Scalar>::fillQ(Qv, vw);

  // fill Qa
  Eigen::Matrix<Scalar, 6, 1> aw;
  aw << Scalar(coeffs()(6)), Scalar(coeffs()(7)), Scalar(coeffs()(8)),
        Scalar(coeffs()(3)), Scalar(coeffs()(4)), Scalar(coeffs()(5));
  Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> Qa = Jl.template block<3,3>(6, 3);
  SE3Tangent<Scalar>::fillQ(Qa, aw);

  return Jl;
}

template <typename _Derived>
typename SE_2_3TangentBase<_Derived>::Jacobian
SE_2_3TangentBase<_Derived>::smallAdj() const
{
  /// this is
  ///       ad(g) = |  Omega  V      0|
  ///               |   0   Omega    0|
  ///               |   0     A  Omega|
  ///
  /// considering vee(log(g)) = (v;w; a)

  Jacobian smallAdj;
  smallAdj.template block<6, 3>(3, 0).setZero();
  smallAdj.template block<6, 3>(0, 6).setZero();
  smallAdj.template block<3,3>(0, 3) = skew(lin());
  smallAdj.template topLeftCorner<3,3>() = skew(ang());
  smallAdj.template block<3,3>(3,3) = smallAdj.template topLeftCorner<3,3>();
  smallAdj.template bottomRightCorner<3,3>() = smallAdj.template topLeftCorner<3,3>();
  smallAdj.template block<3,3>(6, 3) = skew(lin2());
  return smallAdj;
}

// SE_2_3Tangent specific API

template <typename _Derived>
typename SE_2_3TangentBase<_Derived>::LinBlock
SE_2_3TangentBase<_Derived>::lin()
{
  return coeffs().template head<3>();
}

template <typename _Derived>
const typename SE_2_3TangentBase<_Derived>::ConstLinBlock
SE_2_3TangentBase<_Derived>::lin() const
{
  return coeffs().template head<3>();
}

template <typename _Derived>
typename SE_2_3TangentBase<_Derived>::AngBlock
SE_2_3TangentBase<_Derived>::ang()
{
  return coeffs().template segment<3>(3);
}

template <typename _Derived>
const typename SE_2_3TangentBase<_Derived>::ConstAngBlock
SE_2_3TangentBase<_Derived>::ang() const
{
  return coeffs().template segment<3>(3);
}

template <typename _Derived>
typename SE_2_3TangentBase<_Derived>::LinBlock
SE_2_3TangentBase<_Derived>::lin2()
{
  return coeffs().template tail<3>();
}

template <typename _Derived>
const typename SE_2_3TangentBase<_Derived>::ConstLinBlock
SE_2_3TangentBase<_Derived>::lin2() const
{
  return coeffs().template tail<3>();
}


namespace internal {

//! @brief Generator specialization for SE_2_3TangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<SE_2_3TangentBase<Derived>>
{
  static typename SE_2_3TangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    using LieAlg = typename SE_2_3TangentBase<Derived>::LieAlg;
    using Scalar = typename SE_2_3TangentBase<Derived>::Scalar;

    switch (i)
    {
      case 0:
      {
        static const LieAlg E0(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(1), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0)).finished());
        return E0;
      }
      case 1:
      {
        static const LieAlg E1(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(1), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E1;
      }
      case 2:
      {
        static const LieAlg E2(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(1), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E2;
      }
      case 3:
      {
        static const LieAlg E3(
                (LieAlg() << Scalar(0), Scalar(0), Scalar( 0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(-1), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(1), Scalar( 0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar( 0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar( 0), Scalar(0), Scalar(0) ).finished());
        return E3;
      }
      case 4:
      {
        static const LieAlg E4(
                (LieAlg() << Scalar( 0), Scalar(0), Scalar(1), Scalar(0), Scalar(0),
                             Scalar( 0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(-1), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar( 0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar( 0), Scalar(0), Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E4;
      }
      case 5:
      {
        static const LieAlg E5(
                (LieAlg() << Scalar(0), Scalar(-1), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(1), Scalar( 0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar( 0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar( 0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar( 0), Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E5;
      }
      case 6:
      {
        static const LieAlg E6(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(1),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E6;
      }
      case 7:
      {
        static const LieAlg E7(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(1),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E7;
      }
      case 8:
      {
        static const LieAlg E8(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(1),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E8;
      }
      default:
        MANIF_THROW("Index i must be in [0,8]!", invalid_argument);
        break;
    }

    return LieAlg{};
  }
};

//! @brief Random specialization for SE_2_3TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<SE_2_3TangentBase<Derived>>
{
  static void run(SE_2_3TangentBase<Derived>& m)
  {
    // in [-1,1]
    m.coeffs().setRandom();
    // In ball of radius PI
    m.coeffs().template segment<3>(3) = randPointInBall(MANIF_PI).template cast<typename Derived::Scalar>();
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SE_2_3_BASE_H_ */
