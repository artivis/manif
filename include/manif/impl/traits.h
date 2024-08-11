#ifndef _MANIF_MANIF_TRAITS_H_
#define _MANIF_MANIF_TRAITS_H_

#include <array>
#include <type_traits>

namespace manif {

template <typename Derived> struct LieGroupBase;
template <typename Derived> struct TangentBase;

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
 * @note given using FooDouble = Foo<double, Ts...>
 * using FooFloat = typename traitscast<FooDouble, float>::cast;
 */
template <
  template <typename, template<typename> class...> class _Class,
  typename _Scalar,
  typename _NewScalar,
  template<typename> class ... Ts>
struct traitscast<_Class<_Scalar, Ts...>, _NewScalar>
{
  using cast = _Class<_NewScalar, Ts...>;
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


/**
 * @brief intseq: std::integer_sequence-equivalent
 */
template<int ... _I>
struct intseq
{
  using value_type = int;
  static constexpr std::size_t size() noexcept {return sizeof...(_I);}
};

/**
 * @brief concatenate two intseqs
 */
template<typename _Seq1, typename _Seq2>
struct intseq_join;

template<template<int ...> class _IntSeq, int ... _I1, int ... _I2>
struct intseq_join<_IntSeq<_I1...>, _IntSeq<_I2...>>
{
  using type = _IntSeq<_I1..., _I2...>;
};

/**
 * @brief generate intseq 0, 1, ..., N-1
 */
template<std::size_t _N>
struct make_intseq
{
  using type = typename intseq_join<
    typename make_intseq<_N - 1>::type, intseq<_N - 1>
  >::type;
};

template<>
struct make_intseq<0>
{
  using type = intseq<>;
};

template <std::size_t _N>
using make_intseq_t = typename make_intseq<_N>::type;

template <typename T>
constexpr T accumulate(T t) {
  return t;
}

template <typename T, typename... Rest>
constexpr T accumulate(T t, Rest... rest) {
  return t + accumulate(rest...);
}

template <int N, int i, int j, int ...Args> struct compute_indices_gen {
  static constexpr std::array<int, sizeof...(Args)+1> get() {
    return compute_indices_gen<N-1, i+j, Args..., i+j>::get();
  }
};
template <int i, int j, int ...Args> struct compute_indices_gen<1, i, j, Args...> {
  static constexpr std::array<int, sizeof...(Args)+1> get() {
    return { { 0, Args... } };
  }
};
//! @brief Compute indices given block-sizes
template <int... Args> constexpr std::array<int, sizeof...(Args)> compute_indices() {
  return compute_indices_gen<sizeof...(Args), 0, Args...>::get();
}

// traits to check if a template parameter has a LieGroupBase base.
template <typename Derived>
void test_lie_group_base(LieGroupBase<Derived>&& s) {}

template <class, typename T> struct is_manif_group_impl : std::false_type {};
template <typename T> struct
is_manif_group_impl<decltype(test_lie_group_base(std::declval<T>())), T>
  : std::true_type {};
template <typename T> struct is_manif_group
  : is_manif_group_impl<void, T> {};

// traits to check if a template parameter has a TangentBase base.
template <typename Derived>
void test_tangent_base(TangentBase<Derived>&& s) {}

template <class, typename T> struct is_manif_tangent_impl : std::false_type {};
template <typename T> struct
is_manif_tangent_impl<decltype(test_tangent_base(std::declval<T>())), T>
  : std::true_type {};
template <typename T> struct is_manif_tangent
  : is_manif_tangent_impl<void, T> {};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_TRAITS_H_ */
