#ifndef _MANIF_MANIF_MANIFOLD_BASE_H_
#define _MANIF_MANIF_MANIFOLD_BASE_H_

#include "manif/fwd.h"
#include "manif/tangent_base.h"

#include <iostream>
#include <utility>
#include <tuple>

namespace manif
{

template <typename _ManifoldBase> struct ManifoldProperties;

template <class _Derived>
struct ManifoldBase
{
  using Scalar   = typename internal::traits<_Derived>::Scalar;

  using Manifold = typename internal::traits<_Derived>::Manifold;

  static constexpr int Dim     = internal::traits<_Derived>::Dim;
  static constexpr int DoF     = internal::traits<_Derived>::DoF;
  static constexpr int RepSize = internal::traits<_Derived>::RepSize;

  using ManifoldDataType = typename internal::traits<_Derived>::ManifoldDataType;
  using TangentDataType  = typename internal::traits<_Derived>::TangentDataType;

  using JacobianType = typename internal::traits<_Derived>::JacobianType;

  using ManifoldTangentBase = TangentBase<_Derived>;

  using Tangent = typename internal::traits<_Derived>::Tangent;

  /// @todo this is an implicit conversion operator,
  /// evaluate how bad it is to use it.
  operator Manifold&() { return manifold(); }
  operator const Manifold& () const { return manifold(); }

  ManifoldDataType* data()
  {
    return manifold().data();
  }

  const ManifoldDataType* data() const
  {
    return manifold().data();
  }

  void identity()
  {
    //std::cout << "ManifoldBase identity\n";
    manifold().identity();
  }

  void random()
  {
    //std::cout << "ManifoldBase random\n";
    manifold().random();
  }

  Manifold inverse() const
  {
    //std::cout << "ManifoldBase inverse\n";
    return manifold().inverse();
  }

  template <typename _OtherManifold>
  void inverse(_OtherManifold&& m, JacobianType& j) const
  {
    //std::cout << "ManifoldBase inverseWithJacobian\n";
    manifold().inverse(std::forward<_OtherManifold>(m), j);
  }

  Manifold rplus(const Tangent& t) const
  {
    //std::cout << "ManifoldBase rplus\n";
    return manifold().rplus(t);
  }

  Manifold lplus(const Tangent& t) const
  {
    //std::cout << "ManifoldBase lplus\n";
    return manifold().lplus(t);
  }

  template <typename T>
  Manifold plus(T&& t) const
  {
    return manifold().rplus(std::forward<T>(t));
  }

  Manifold rminus(const Manifold& m) const
  {
    return manifold().rminus(m);
  }

  Manifold lminus(const Manifold& m) const
  {
    return manifold().lminus(m);
  }

  /*
  template <typename T>
  Manifold minus(T&& t) const
  {
    return manifold().rminus(std::forward<T>(t));
  }
  */

  ManifoldTangentBase lift() const
  {
    return manifold().lift();
  }

  /*
  LieType lie() const
  {
    return manifold().lie();
  }
  */

  Manifold compose(const Manifold& m) const
  {
    return manifold().compose(m);
  }

  Manifold between(const Manifold& m) const
  {
    return manifold().inverse().compose(m);
  }

  /*
  Manifold interpolate()
  {
    return manifold().interpolate();
  }
  */

  /// Some operators

  template <typename T>
  Manifold operator +(T&& t) const
  {
    return manifold().rplus(std::forward<T>(t));
  }

  template <typename T>
  Manifold& operator +=(T&& t)
  {
    manifold() = manifold().rplus(std::forward<T>(t));
    return manifold();
  }

  template <typename T>
  Manifold operator *(T&& m) const
  {
    return manifold().compose(std::forward<T>(m));
  }

  template <typename T>
  Manifold& operator *=(T&& m)
  {
    manifold() = manifold().compose(std::forward<T>(m));
    return manifold();
  }

  /// Some static helpers

  static Manifold Identity()
  {
    static Manifold m; m.identity();
    return m;
  }

  static Manifold Random()
  {
    static Manifold m; m.random();
    return m;
  }

  static ManifoldTangentBase Inverse(const Manifold& m)
  {
    return m.inverse();
  }

  static Manifold Rplus(const Manifold& m, const ManifoldTangentBase& t)
  {
    return m.plus(t);
  }

  static Manifold Lplus(const ManifoldTangentBase& t, const Manifold& m)
  {
    return t.plus(m);
  }

  static Manifold Rminus(const Manifold& m0, const Manifold& m1)
  {
    return m0.rminus(m1);
  }

  static Manifold Lminus(const Manifold& m0, const Manifold& m1)
  {
    return m0.lminus(m1);
  }

  static ManifoldTangentBase Lift(const Manifold& m)
  {
    return m.lift();
  }

  static Manifold Retract(const ManifoldTangentBase& t)
  {
    return t.retract();
  }

  /*
  static LieType Lie(const Manifold& m)
  {
    return m.lie();
  }

  static LieType Lie(const ManifoldTangentBase& t)
  {
    return t.lie();
  }
  */

  static Manifold Compose(const Manifold& m0, const Manifold& m1)
  {
    return m0.compose(m1);
  }

  static Manifold Between(const Manifold& m0, const Manifold& m1)
  {
    return m0.between(m1);
  }

  /// static helpers with Jacobians

  template <typename _Jacobian>
  static void Inverse(const Manifold& m, Manifold& minv, _Jacobian& jac)
  {
    minv = m.inverse();
//    InverseJacobianHelper<Manifold>::compute/*<_Jacobian>*/(m, minv, jac);
  }

private:

  Manifold& manifold() { return *static_cast< Manifold* >(this); }
  const Manifold& manifold() const { return *static_cast< const Manifold* >(this); }
};

} /* namespace manif */

#endif /* _MANIF_MANIF_MANIFOLD_BASE_H_ */
