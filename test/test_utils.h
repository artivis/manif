#ifndef _MANIF_MANIF_TEST_UTILS_H_
#define _MANIF_MANIF_TEST_UTILS_H_

#include "manif/impl/manifold_base.h"
#include "manif/impl/utils.h"

#include <random>

#define EXPECT_ANGLE_NEAR(e, a, eps) \
  EXPECT_LT(pi2pi(e-a), eps)

namespace manif {

template <typename _Scalar = double>
class GaussianNoiseGenerator
{
  using Clock = std::chrono::system_clock;
  using Scalar = _Scalar;

public:

  GaussianNoiseGenerator(const Scalar mean,
                         const Scalar std)
    : re_(Clock::now().time_since_epoch().count())
    , distr_(mean, std)
  {
    //
  }

  Scalar noise()
  {
    return distr_(re_);
  }

  Scalar operator()()
  {
    return noise();
  }

protected:

  std::default_random_engine re_;
  std::normal_distribution<Scalar> distr_;
};

template <typename _Derived>
const typename _Derived::DataType&
callCoeffs(const ManifoldBase<_Derived>& manifold)
{
  return manifold.coeffs();
}

template <typename _Derived>
const typename _Derived::DataType&
callCoeffs(const TangentBase<_Derived>& tangent)
{
  return tangent.coeffs();
}

template <typename _Derived>
typename _Derived::Transformation
callTransform(const ManifoldBase<_Derived>& manifold)
{
  return manifold.transform();
}

template <typename _Derived>
typename _Derived::Rotation
callRotation(const ManifoldBase<_Derived>& manifold)
{
  return manifold.rotation();
}

template <typename _Derived>
void
callIdentity(ManifoldBase<_Derived>& manifold)
{
  manifold.identity();
}

template <typename _Derived>
void
callZero(TangentBase<_Derived>& tangent)
{
  tangent.zero();
}

template <typename _Derived>
void
callRandom(ManifoldBase<_Derived>& manifold)
{
  manifold.random();
}

template <typename _Derived>
void
callRandom(TangentBase<_Derived>& tangent)
{
  tangent.random();
}

template <typename _Derived>
typename _Derived::Manifold
callInverse(const ManifoldBase<_Derived>& manifold)
{
  return manifold.inverse();
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::Manifold
callRplus(const ManifoldBase<_DerivedMan>& manifold,
          const TangentBase<_DerivedTan>& tangent)
{
  return manifold.rplus(tangent);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::Manifold
callLplus(const ManifoldBase<_DerivedMan>& manifold,
          const TangentBase<_DerivedTan>& tangent)
{
  return manifold.lplus(tangent);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::Manifold
callPlus(const ManifoldBase<_DerivedMan>& manifold,
         const TangentBase<_DerivedTan>& tangent)
{
  return manifold.plus(tangent);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
callRminus(const ManifoldBase<_Derived0>& manifold_lhs,
           const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs.rminus(manifold_rhs);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
callLminus(const ManifoldBase<_Derived0>& manifold_lhs,
           const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs.lminus(manifold_rhs);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
callMinus(const ManifoldBase<_Derived0>& manifold_lhs,
          const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs.minus(manifold_rhs);
}

template <typename _Derived>
typename _Derived::Tangent
callLift(const ManifoldBase<_Derived>& manifold)
{
  return manifold.lift();
}

template <typename _Derived>
typename _Derived::Manifold
callRetract(const TangentBase<_Derived>& tangent)
{
  return tangent.retract();
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Manifold
callCompose(const ManifoldBase<_Derived0>& manifold_lhs,
            const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs.compose(manifold_rhs);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Manifold
callBetween(const ManifoldBase<_Derived0>& manifold_lhs,
            const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs.between(manifold_rhs);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::Manifold
callOpPlus(const ManifoldBase<_DerivedMan>& manifold,
           const TangentBase<_DerivedTan>& tangent)
{
  return manifold + tangent;
}

template <typename _DerivedMan, typename _DerivedTan>
void
callOpPlusEq(ManifoldBase<_DerivedMan>& manifold,
             const TangentBase<_DerivedTan>& tangent)
{
  manifold += tangent;
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
callOpMinus(const ManifoldBase<_Derived0>& manifold_lhs,
            const ManifoldBase<_Derived0>& manifold_rhs)
{
  return manifold_lhs - manifold_rhs;
}

template <typename _Derived0, typename _Derived1>
const typename _Derived0::Manifold
callOpTime(const ManifoldBase<_Derived0>& manifold_lhs,
           const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs * manifold_rhs;
}

template <typename _Derived0, typename _Derived1>
void
callOpTimeEq(ManifoldBase<_Derived0>& manifold_lhs,
             const ManifoldBase<_Derived1>& manifold_rhs)
{
  manifold_lhs *= manifold_rhs;
}

/// with Jacs

template <typename _Derived>
const typename _Derived::DataType*
callInverseWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callRplusWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callLplusWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callPlusWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callRminusWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callLminusWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callMinusWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callLiftWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
void
callRetractWithJac(const TangentBase<_Derived>& tangent,
                   typename _Derived::Manifold& manifold,
                   typename _Derived::Jacobian& J)
{
  return tangent.retract(manifold, J);
}

template <typename _Derived>
const typename _Derived::DataType*
callComposeWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callBetweenWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_TEST_UTILS_H_ */
