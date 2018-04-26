#ifndef _MANIF_MANIF_MANIFOLD_BASE_H_
#define _MANIF_MANIF_MANIFOLD_BASE_H_

#include "manif/impl/fwd.h"
#include "manif/impl/tangent_base.h"

#include "lspdlog/logging.h"

namespace manif
{


namespace internal
{

template<class, typename T> struct has_rplus_impl : std::false_type {};
template<typename T> struct
has_rplus_impl<decltype( void(std::declval<T>().rplus(typename T::Tangent{})) ), T> : std::true_type {};
template<typename T> struct has_rplus : has_rplus_impl<void, T> {};

template<class, typename T> struct has_inverse_impl : std::false_type {};
template<typename T> struct
has_inverse_impl<decltype( void(std::declval<T>().inverse()) ), T> : std::true_type {};
template<typename T> struct has_inverse : has_inverse_impl<void, T> {};

template<class, typename T> struct has_foo_impl : std::false_type {};
template<typename T> struct
has_foo_impl<decltype( void(std::declval<T>().foo()) ), T> : std::true_type {};
template<typename T> struct has_foo : has_foo_impl<void, T> {};

template<class, typename T> struct implement_inverse_impl : std::false_type {};

template<typename T> struct
implement_inverse_impl<decltype(
    typename std::enable_if<
//    std::true_type::value &&
      std::is_same<typename T::Manifold (T::Base::*)() const, decltype(&T::ttt)>::value
      , void
    >::type()
    ), T> : std::true_type
{};

template<typename T> struct implement_inverse : implement_inverse_impl<void, T> {};

// Check for func in base
template< class T >
struct CheckForFoo
{
  typedef char(&YesType)[1];
  typedef char(&NoType)[2];

  template< class T2 > static YesType Test(
      typename std::enable_if<
      !has_foo<T>::value
//      &&
//      std::is_same<typename T::Manifold (T::Base::Base::*)(), decltype(&T2::foo)>::value,
//      !std::is_same<typename T::Manifold (T::*)() const, decltype(&T2::bar)>::value
      ,
      int
      >::type );
  template< class T2 > static NoType Test( ... );
  static const bool value = sizeof(Test<T>(0))==sizeof(YesType);
};

template< class T >
struct implement_rplus
{
  typedef char(&YesType)[1];
  typedef char(&NoType)[2];

  template< class T2 > static YesType Test(
      typename std::enable_if<
      std::is_same<typename T::Manifold (T::*)(const typename T::Tangent&)const, decltype(&T2::rplus)>::value,
      int
      >::type );
  template< class T2 > static NoType  Test( ... );
  static const bool value = sizeof(Test<T>(0))==sizeof(YesType);
};

} /* namespace internal */




template <class _Derived>
struct ManifoldBase
{
  using Scalar   = typename internal::traits<_Derived>::Scalar;

  using Manifold = typename internal::traits<_Derived>::Manifold;

  static constexpr int Dim     = internal::traits<_Derived>::Dim;
  static constexpr int DoF     = internal::traits<_Derived>::DoF;
  static constexpr int N       = internal::traits<_Derived>::N;
  static constexpr int RepSize = internal::traits<_Derived>::RepSize;

  using ManifoldDataType = typename internal::traits<_Derived>::ManifoldDataType;

  using ManifoldTangentBase = TangentBase<_Derived>;

  using Tangent = typename internal::traits<_Derived>::Tangent;

  using Jacobian = typename internal::traits<_Derived>::Jacobian;

  using Transformation = typename internal::traits<_Derived>::Transformation;

  using Rotation = typename internal::traits<_Derived>::Rotation;

  /// @todo this is an implicit conversion operator,
  /// evaluate how bad it is to use it.
  operator Manifold&() { return derived(); }
  operator const Manifold& () const { return derived(); }

protected:

  ManifoldDataType* data();

public:

  const ManifoldDataType* data() const;

  Transformation matrix() const;

  Rotation rotation() const;

  void identity();

  void random();

  Manifold inverse() const;

  Manifold rplus(const Tangent& t) const;
  Manifold lplus(const Tangent& t) const;

  /**
   * @brief plus, calls rplus
   * @see rplus
   */
  Manifold plus(const Tangent& t) const;

  Tangent rminus(const Manifold& m) const;
  Tangent lminus(const Manifold& m) const;
  Tangent minus(const Manifold& m) const;

  Tangent lift() const;

  // LieType lie() const {return derived().lie();}

  Manifold compose(const Manifold& m) const;

  Manifold between(const Manifold& m) const;

//  Manifold interpolate() {return derived().interpolate();}

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
   * @brief operator -, rminus
   * @param m
   * @return
   * @see rminus
   */
  Tangent operator -(const Manifold& m) const;

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

  void inverse(Manifold& m, Jacobian& J) const;

  void rplus(const Tangent& t, Manifold& m,
             Jacobian& J_c_a, Jacobian& J_c_b) const;

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

  _Derived& derived() { return *static_cast< _Derived* >(this); }
  const _Derived& derived() const { return *static_cast< const _Derived* >(this); }
};

template <typename _Derived>
typename ManifoldBase<_Derived>::ManifoldDataType*
ManifoldBase<_Derived>::data()
{
  return derived().data();
}

template <typename _Derived>
const typename ManifoldBase<_Derived>::ManifoldDataType*
ManifoldBase<_Derived>::data() const
{
  return derived().data();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Transformation
ManifoldBase<_Derived>::matrix() const
{
  return derived().matrix();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Rotation
ManifoldBase<_Derived>::rotation() const
{
  return derived().rotation();
}

template <typename _Derived>
void ManifoldBase<_Derived>::identity()
{
  //std::cout << "ManifoldBase identity\n";
  derived().identity();
}

template <typename _Derived>
void ManifoldBase<_Derived>::random()
{
  //std::cout << "ManifoldBase random\n";
  derived().random();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::inverse() const
{
  static_assert(internal::has_inverse<typename _Derived::Base>::value,
                "Derived class must implement 'inverse' !");

  return derived().inverse();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::rplus(const Tangent& t) const
{
  static_assert(internal::has_rplus<typename _Derived::Base>::value,
                "Derived class must implement 'rplus' !");

  return compose(t.retract());
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::lplus(const Tangent& t) const
{
  //std::cout << "ManifoldBase lplus\n";
  return t.retract().compose(derived());
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::plus(const Tangent& t) const
{
  return rplus(t);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::rminus(const Manifold& m) const
{
  return m.inverse().compose(derived()).lift();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::lminus(const Manifold& m) const
{
  return derived().inverse().compose(m).lift();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::minus(const Manifold& m) const
{
  return rminus(m);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::lift() const
{
  return derived().lift();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::compose(const Manifold& m) const
{
  return derived().compose(m);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::between(const Manifold& m) const
{
  return derived().inverse().compose(m);
}

/// Operators

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::operator +(const Tangent& t) const
{
  return derived().rplus(t);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold&
ManifoldBase<_Derived>::operator +=(const Tangent& t)
{
  derived() = derived().rplus(t);
  return derived();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::operator -(const Manifold& m) const
{
  return derived().rminus(m);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::operator *(const Manifold& m) const
{
  return derived().compose(m);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold&
ManifoldBase<_Derived>::operator *=(const Manifold& m)
{
  derived() = derived().compose(m);
  return derived();
}

/// Jacs

template <typename _Derived>
void ManifoldBase<_Derived>::inverse(Manifold& m, Jacobian& j) const
{
  derived().inverse(m, j);
}

/// Utils

template <typename _Stream, typename _Derived>
_Stream& operator << (
    _Stream& s,
    const manif::ManifoldBase<_Derived>& m)
{
  s << *m.data();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_MANIFOLD_BASE_H_ */
