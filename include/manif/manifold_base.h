#ifndef _MANIF_MANIF_MANIFOLD_BASE_H_
#define _MANIF_MANIF_MANIFOLD_BASE_H_

#include "manif/fwd.h"
#include "manif/tangent_base.h"

#include "lspdlog/logging.h"

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

  using ManifoldTangentBase = TangentBase<_Derived>;

  using Tangent = typename internal::traits<_Derived>::Tangent;

  using JacobianMtoM = typename internal::traits<_Derived>::JacobianMtoM;
  using JacobianMtoT = typename internal::traits<_Derived>::JacobianMtoT;

  using Transformation = typename internal::traits<_Derived>::Transformation;

  using Rotation = typename internal::traits<_Derived>::Rotation;

  /// @todo this is an implicit conversion operator,
  /// evaluate how bad it is to use it.
  operator Manifold&() { return manifold(); }
  operator const Manifold& () const { return manifold(); }

  ManifoldDataType* data();

  const ManifoldDataType* data() const;

  Transformation matrix() const;

  Rotation rotation() const;

  void identity();

  void random();

  Manifold inverse() const;

  Manifold rplus(const Tangent& t) const;

  Manifold lplus(const Tangent& t) const;

  Manifold plus(const Tangent& t) const;

  Manifold rminus(const Manifold& m) const;

  Manifold lminus(const Manifold& m) const;

  /*
  template <typename T>
  Manifold minus(T&& t) const
  {
    return manifold().rminus(std::forward<T>(t));
  }
  */

  Tangent lift() const;

  /*
  LieType lie() const
  {
    return manifold().lie();
  }
  */

  Manifold compose(const Manifold& m) const;

  Manifold between(const Manifold& m) const;

  /*
  Manifold interpolate()
  {
    return manifold().interpolate();
  }
  */

  /// Some operators

  /**
   * @brief operator +, rplus
   * @param t
   * @return
   * @see rplus
   */
  Manifold operator +(const Tangent& t) const;

  /**
   * @brief operator +=, in-place rplus
   * @param t
   * @return
   * @see rplus
   */
  Manifold& operator +=(const Tangent& t);

  /**
   * @brief operator *, compose
   * @param m
   * @return
   * @see compose
   */
  Manifold operator *(const Manifold& m) const;

  /**
   * @brief operator *=, in-place compose
   * @param m
   * @return
   * @see compose
   */
  Manifold& operator *=(const Manifold& m);

  /// Jacs

  void inverse(Manifold& m, JacobianMtoM& J) const;

  void rplus(const Tangent& t, Manifold& m,
             JacobianMtoM& J_c_a, JacobianMtoM& J_c_b) const;

  /// Some static helpers

  static Manifold Identity()
  {
    /// @todo how to optimize .identity() call away ?
    static Manifold m; m.identity();
    return m;
  }

  static Manifold Random()
  {
    static Manifold m; m.random();
    return m;
  }

  static Tangent Inverse(const Manifold& m)
  {
    return m.inverse();
  }

  static Manifold Rplus(const Manifold& m, const Tangent& t)
  {
    return m.plus(t);
  }

  static Manifold Lplus(const Tangent& t, const Manifold& m)
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

  static Tangent Lift(const Manifold& m)
  {
    return m.lift();
  }

  static Manifold Retract(const Tangent& t)
  {
    return t.retract();
  }

  /*
  static LieType Lie(const Manifold& m)
  {
    return m.lie();
  }

  static LieType Lie(const Tangent& t)
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
//    minv = m.inverse();
//    InverseJacobianHelper<Manifold>::compute/*<_Jacobian>*/(m, minv, jac);
  }

private:

  Manifold& manifold() { return *static_cast< Manifold* >(this); }
  const Manifold& manifold() const { return *static_cast< const Manifold* >(this); }
};

template <typename _Derived>
typename ManifoldBase<_Derived>::ManifoldDataType*
ManifoldBase<_Derived>::data()
{
  return manifold().data();
}

template <typename _Derived>
const typename ManifoldBase<_Derived>::ManifoldDataType*
ManifoldBase<_Derived>::data() const
{
  return manifold().data();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Transformation
ManifoldBase<_Derived>::matrix() const
{
  return manifold().matrix();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Rotation
ManifoldBase<_Derived>::rotation() const
{
  return manifold().rotation();
}

template <typename _Derived>
void ManifoldBase<_Derived>::identity()
{
  //std::cout << "ManifoldBase identity\n";
  manifold().identity();
}

template <typename _Derived>
void ManifoldBase<_Derived>::random()
{
  //std::cout << "ManifoldBase random\n";
  manifold().random();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::inverse() const
{
  //std::cout << "ManifoldBase inverse\n";
  return manifold().inverse();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::rplus(const Tangent& t) const
{
  //std::cout << "ManifoldBase rplus\n";
  return manifold().rplus(t);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::lplus(const Tangent& t) const
{
  //std::cout << "ManifoldBase lplus\n";
  return manifold().lplus(t);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::plus(const Tangent& t) const
{
  return manifold().rplus(t);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::rminus(const Manifold& m) const
{
  return manifold().rminus(m);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::lminus(const Manifold& m) const
{
  return manifold().lminus(m);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::lift() const
{
  return manifold().lift();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::compose(const Manifold& m) const
{
  return manifold().compose(m);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::between(const Manifold& m) const
{
  return manifold().inverse().compose(m);
}

/// Operators

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::operator +(const Tangent& t) const
{
  return manifold().rplus(t);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold&
ManifoldBase<_Derived>::operator +=(const Tangent& t)
{
  manifold() = manifold().rplus(t);
  return manifold();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::operator *(const Manifold& m) const
{
  return manifold().compose(m);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold&
ManifoldBase<_Derived>::operator *=(const Manifold& m)
{
  manifold() = manifold().compose(m);
  return manifold();
}

/// Jacs

template <typename _Derived>
void ManifoldBase<_Derived>::inverse(Manifold& m, JacobianMtoM& j) const
{
  manifold().inverse(m, j);
}

} /* namespace manif */

#endif /* _MANIF_MANIF_MANIFOLD_BASE_H_ */
