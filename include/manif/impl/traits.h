#ifndef _MANIF_MANIF_TRAITS_H_
#define _MANIF_MANIF_TRAITS_H_

#include <type_traits>

namespace manif {
namespace internal {

//template <typename T>
//inline constexpr bool constexpr_false() {
//    return false;
//}

/// @note Triggers static_asserts only if a template is instantiated
template <typename... T>
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

template <class T>
using remove_cr = std::remove_const<typename std::remove_reference<T>::type>;

template <class T>
using remove_cr_t = typename remove_cr<T>::type;

// template <typename...> struct get_base;
//
// template <template <typename _Scalar> class DerivedBase>
// struct get_base
// {
//   using type = void;
// }

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_TRAITS_H_ */
