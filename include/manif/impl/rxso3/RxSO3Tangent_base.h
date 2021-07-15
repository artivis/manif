#ifndef _MANIF_MANIF_RxSO3TANGENT_BASE_H_
#define _MANIF_MANIF_RxSO3TANGENT_BASE_H_

#include "manif/impl/rxso3/RxSO3_properties.h"
#include "manif/impl/tangent_base.h"

#include "manif/impl/so3/SO3Tangent_map.h"

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the RxSO3 tangent.
 * @note See Appendix B.
 */
template <typename _Derived>
struct RxSO3TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = RxSO3TangentBase<_Derived>;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using AngBlock = typename DataType::template FixedSegmentReturnType<3>::Type;
  using ConstAngBlock = typename DataType::template ConstFixedSegmentReturnType<3>::Type;

  // Tangent common API

  using Base::coeffs;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(RxSO3TangentBase)

public:

  MANIF_TANGENT_ML_ASSIGN_OP(RxSO3TangentBase)

  /**
   * @brief Hat operator of RxSO3.
   * @return An element of the Lie algebra rxso3.
   */
  LieAlg hat() const;

  /**
   * @brief Get the RxSO3 element.
   * @param[out] -optional- J_m_t Jacobian of the RxSO3 element wrt this.
   * @return The RxSO3 element.
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
   * Get the right Jacobian of RxSO3.
   */
  Jacobian rjac() const;

  /**
   * Get the left Jacobian of RxSO3.
   */
  Jacobian ljac() const;

  /**
   * Get the inverse of the right Jacobian of RxSO3.
   * @see rjac.
   */
  Jacobian rjacinv() const;

  /**
   * Get the inverse of the left Jacobian of RxSO3.
   * @see ljac.
   */
  Jacobian ljacinv() const;

  /**
   * @brief
   * @return
   */
  Jacobian smallAdj() const;

  // RxSO3Tangent specific API

  //! @brief
  Scalar s() const;
  //! @brief
  Scalar x() const;
  //! @brief
  Scalar y() const;
  //! @brief
  Scalar z() const;

  //! @brief Get the angular part.
  AngBlock ang();
  const ConstAngBlock ang() const;

// protected:

  Eigen::Map<SO3Tangent<Scalar>> asSO3();
  const Eigen::Map<const SO3Tangent<Scalar>> asSO3() const;
};

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::LieGroup
RxSO3TangentBase<_Derived>::exp(OptJacobianRef J_m_t) const
{
  using std::exp;

  if (J_m_t) {
    *J_m_t = rjac();
  }

  return LieGroup((
    typename LieGroup::DataType() << asSO3().exp().coeffs(), exp(s())
  ).finished());
}

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::LieGroup
RxSO3TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::Jacobian
RxSO3TangentBase<_Derived>::rjac() const
{
  Jacobian rjac = Jacobian::Identity();
  rjac.template topLeftCorner<3, 3>() = asSO3().rjac();
  return rjac;
}

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::Jacobian
RxSO3TangentBase<_Derived>::ljac() const
{
  Jacobian ljac = Jacobian::Identity();
  ljac.template topLeftCorner<3, 3>() = asSO3().ljac();
  return ljac;
}

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::Jacobian
RxSO3TangentBase<_Derived>::rjacinv() const
{
  Jacobian rjacinv = Jacobian::Identity();
  rjacinv.template topLeftCorner<3, 3>() = asSO3().rjacinv();
  return rjacinv;
}

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::Jacobian
RxSO3TangentBase<_Derived>::ljacinv() const
{
  Jacobian ljacinv = Jacobian::Identity();
  ljacinv.template topLeftCorner<3, 3>() = asSO3().ljacinv();
  return ljacinv;
}

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::Jacobian
RxSO3TangentBase<_Derived>::smallAdj() const
{
  Jacobian ad = Jacobian::Zero();
  ad.template topLeftCorner<3,3>() = skew(ang());
  return ad;
}

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::LieAlg
RxSO3TangentBase<_Derived>::hat() const
{
  return s() * LieAlg::Identity() + skew(ang());
}

// RxSO3Tangent specifics

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::Scalar
RxSO3TangentBase<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::Scalar
RxSO3TangentBase<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::Scalar
RxSO3TangentBase<_Derived>::z() const
{
  return coeffs().z();
}

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::Scalar
RxSO3TangentBase<_Derived>::s() const
{
  return coeffs().w();
}

template <typename _Derived>
typename RxSO3TangentBase<_Derived>::AngBlock
RxSO3TangentBase<_Derived>::ang()
{
  return coeffs().template head<3>();
}

template <typename _Derived>
const typename RxSO3TangentBase<_Derived>::ConstAngBlock
RxSO3TangentBase<_Derived>::ang() const
{
  return coeffs().template head<3>();
}

template <typename _Derived>
const Eigen::Map<const SO3Tangent<typename RxSO3TangentBase<_Derived>::Scalar>>
RxSO3TangentBase<_Derived>::asSO3() const
{
  return Eigen::Map<const SO3Tangent<Scalar>>(coeffs().data());
}

template <typename _Derived>
Eigen::Map<SO3Tangent<typename RxSO3TangentBase<_Derived>::Scalar>>
RxSO3TangentBase<_Derived>::asSO3()
{
  return Eigen::Map<SO3Tangent<Scalar>>(coeffs().data());
}

namespace internal {

//! @brief Generator specialization for RxSO3TangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<RxSO3TangentBase<Derived>>
{
  static typename RxSO3TangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    using LieAlg = typename RxSO3TangentBase<Derived>::LieAlg;
    using Scalar = typename RxSO3TangentBase<Derived>::Scalar;

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
      case 3:
      {
        static const LieAlg E3(
              (LieAlg() << Scalar(1), Scalar(0), Scalar(0),
                           Scalar(0), Scalar(1), Scalar(0),
                           Scalar(0), Scalar(0), Scalar(1) ).finished());
        return E3;
      }
      default:
        MANIF_THROW("Index i must be in [0,3]!", invalid_argument);
        break;
    }

    return LieAlg{};
  }
};

//! @brief Random specialization for RxSO3TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<RxSO3TangentBase<Derived>>
{
  static void run(RxSO3TangentBase<Derived>& m)
  {
    using Scalar = typename Derived::Scalar;
    // In ball of radius PI
    m.ang() = randPointInBall(MANIF_PI).template cast<Scalar>();
    m.coeffs()(3) = Eigen::internal::random<Scalar>();
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_RxSO3TANGENT_BASE_H_
