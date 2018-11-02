#ifndef _MANIF_MANIF_TRAITS_H_
#define _MANIF_MANIF_TRAITS_H_

#include <type_traits>

namespace manif {
namespace internal {

template <typename T> struct traits;

/// @note the following is from the Eigen library

// here we say once and for all that traits<const T> == traits<T>
// When constness must affect traits, it has to be constness on
// template parameters on which T itself depends.
// For example, traits<Map<const T> > != traits<Map<T> >, but
//              traits<const Map<T> > == traits<Map<T> >
template<typename T> struct traits<const T> : traits<T> {};

/// @brief ManifoldProperties
/**
 * @brief ManifoldProperties, a traits for defining some
 * manifold properties. E.g. Space dimension, Degrees of Freedom ...
 */
template <typename _ManifoldBase> struct ManifoldProperties;

//template <template <typename _Derived> class _Base>
//struct TangentBaseHelper
//{
//  using Base = TangentBase<_Derived>;
//};

template <typename _Class, typename _NewScalar>
struct traitscast;

template <template <typename> class _Class, typename _NewScalar, typename _Scalar>
struct traitscast<_Class<_Scalar>, _NewScalar>
{
  using cast = _Class<_NewScalar>;
};

////////////////////////////

template<class, typename T> struct has_rjacinv_impl : std::false_type {};
template<typename T> struct
has_rjacinv_impl<decltype( void(std::declval<T>().rjacinv()) ), T> : std::true_type {};
template<typename T> struct has_rjacinv : has_rjacinv_impl<void, T> {};

template<class, typename T> struct has_ljacinv_impl : std::false_type {};
template<typename T> struct
has_ljacinv_impl<decltype( void(std::declval<T>().ljacinv()) ), T> : std::true_type {};
template<typename T> struct has_ljacinv : has_ljacinv_impl<void, T> {};

} /* namespace internal */
} /* namespace manif */

////// Garbage / messing arround

namespace manif {
namespace internal {

template<class, typename T> struct has_rplus_impl : std::false_type {};
template<typename T> struct
has_rplus_impl<decltype( void(std::declval<T>().rplus(typename T::Tangent{})) ), T> : std::true_type {};
template<typename T> struct has_rplus : has_rplus_impl<void, T> {};

template<class, typename T> struct has_inverse_impl : std::false_type {};
template<typename T> struct
has_inverse_impl<decltype( void(std::declval<T>().inverse()) ), T> : std::true_type {};
template<typename T> struct has_inverse : has_inverse_impl<void, T> {};

//template<class, typename T> struct has_foo_impl : std::false_type {};
//template<typename T> struct
//has_foo_impl<decltype( void(std::declval<T>().foo()) ), T> : std::true_type {};
//template<typename T> struct has_foo : has_foo_impl<void, T> {};

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
//template< class T >
//struct CheckForFoo
//{
//  typedef char(&YesType)[1];
//  typedef char(&NoType)[2];

//  template< class T2 > static YesType Test(
//      typename std::enable_if<
//      !has_foo<T>::value
////      &&
////      std::is_same<typename T::Manifold (T::Base::Base::*)(), decltype(&T2::foo)>::value,
////      !std::is_same<typename T::Manifold (T::*)() const, decltype(&T2::bar)>::value
//      ,
//      int
//      >::type );
//  template< class T2 > static NoType Test( ... );
//  static const bool value = sizeof(Test<T>(0))==sizeof(YesType);
//};

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

template<class, typename T> struct has_foo_impl : std::false_type {};
template<typename T> struct
has_foo_impl<decltype( void(std::declval<const typename T::Base>().foo()) ), T> : std::true_type {};
template<typename T> struct has_foo : has_foo_impl<void, T> {};

template< class T >
struct implement_foo
{
  typedef char(&YesType)[1];
  typedef char(&NoType)[2];

  template< class T2 > static YesType Test(
      typename std::enable_if<
      std::is_same<typename T::Manifold (T::*)()const, decltype(&T2::foo)>::value,
      int
      >::type );
  template< class T2 > static NoType  Test( ... );
  static const bool value = sizeof(Test<T>(0))==sizeof(YesType);
};

//template<class, typename T, class...> struct has_foo_impl : std::false_type {};
//template<typename T, class... Args> struct
//has_foo_impl<decltype(
//      void(std::declval<const T>().foo(std::forward<Args...>(/*std::declval<Args&&...>()*/Args{}...)))
////      void(std::declval<T>().inverse(std::forward<Args...>(/*std::declval<Args...>()*/Args()...)))
//    ), T> : std::true_type {};
//template<typename T, class... Args> struct has_foo : has_foo_impl<void, T, Args...> {};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_TRAITS_H_ */
