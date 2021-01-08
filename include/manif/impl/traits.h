#ifndef _MANIF_MANIF_TRAITS_H_
#define _MANIF_MANIF_TRAITS_H_

#include <type_traits>

namespace manif {
namespace internal {

//template <typename T>
//inline constexpr bool constexpr_false() {
//    return false;
//}

template <typename T>
struct constexpr_false : std::false_type {};

template <typename T> struct traits;

/// @note the following is from the Eigen library
/// here we say once and for all that traits<const T> == traits<T>
///
/// When constness must affect traits, it has to be constness on
/// template parameters on which T itself depends.
/// For example, traits<Map<const T> > != traits<Map<T> >, but
///              traits<const Map<T> > == traits<Map<T> >
template<typename T> struct traits<const T> : traits<T> {};

/**
 * @brief LieGroupProperties, a traits for defining some
 * group properties. E.g. Space dimension, Degrees of Freedom ...
 */
template <typename _LieGroupBase> struct LieGroupProperties;

//! @brief A traits helper to cast
//!
//! @note e.g. template <typename NewScalar> using LieGroupTemplate = typename traitscast<LieGroup, NewScalar>::cast;
template <typename _Class, typename _NewScalar>
struct traitscast;

/**
 * @brief Traits to change the scalar type of a template class
 * @note given using FooDouble = Foo<double>
 * using FooFloat = typename traitscast<FooDouble, float>::cast;
 */
template <template <typename> class _Class, typename _NewScalar, typename _Scalar>
struct traitscast<_Class<_Scalar>, _NewScalar>
{
  using cast = _Class<_NewScalar>;
};

/**
 * @brief Specialization for Rn.
 */
template <
  template <typename, unsigned int> class _Class,
  typename _NewScalar,
  typename _Scalar,
  unsigned int _Dim>
struct traitscast<_Class<_Scalar, _Dim>, _NewScalar>
{
  using cast = _Class<_NewScalar, _Dim>;
};

//! @brief A traits for detecting AutoDiff scalar types
template <typename Scalar> struct is_ad : std::integral_constant<bool, false> { };

//! @brief Has function 'rjacinv' traits
template<class, typename T> struct has_rjacinv_impl : std::false_type {};
template<typename T> struct
has_rjacinv_impl<decltype( void(std::declval<T>().rjacinv()) ), T> : std::true_type {};
template<typename T> struct has_rjacinv : has_rjacinv_impl<void, T> {};

//! @brief Has function 'ljacinv' traits
template<class, typename T> struct has_ljacinv_impl : std::false_type {};
template<typename T> struct
has_ljacinv_impl<decltype( void(std::declval<T>().ljacinv()) ), T> : std::true_type {};
template<typename T> struct has_ljacinv : has_ljacinv_impl<void, T> {};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_TRAITS_H_ */
