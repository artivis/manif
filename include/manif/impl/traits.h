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
  using type = typename intseq_join<typename make_intseq<_N - 1>::type, intseq<_N - 1>>::type;
};

template<>
struct make_intseq<0>
{
  using type = intseq<>;
};

/**
 * @brief extract intseq element
 */
template<std::size_t _Idx, typename _Seq>
struct intseq_element;

template<std::size_t _Idx, template<int ...> class _IntSeq, int Head, int ... Tail>
struct intseq_element<_Idx, _IntSeq<Head, Tail...>>
  : public intseq_element<_Idx - 1, _IntSeq<Tail...>>
{};

template<template<int ...> class _IntSeq, int Head, int ... Tail>
struct intseq_element<0, _IntSeq<Head, Tail ...>>
{
  static constexpr int value = Head;
};

/**
 * @brief sum an intseq
 */
template<typename _Seq>
struct intseq_sum;

template<template<int ...> class _IntSeq, int I, int ... Is>
struct intseq_sum<_IntSeq<I, Is...>>
{
  static constexpr int value = I + intseq_sum<_IntSeq<Is...>>::value;
};

template<template<int ...> class _IntSeq>
struct intseq_sum<_IntSeq<>>
{
  static constexpr int value = 0;
};

/**
 * @brief prefix-sum an intseq
 */
template<typename _Collected, typename _Remaining, int Sum>
struct intseq_psum;

template<template<int ...> class _IntSeq, int... _Cur, int _Sum>
struct intseq_psum<_IntSeq<_Cur...>, _IntSeq<>, _Sum>
{
  using type = _IntSeq<_Cur...>;
};

template<template<int ...> class _IntSeq, int _First, int _Sum, int... _Cur, int... _Rem>
struct intseq_psum<_IntSeq<_Cur...>, _IntSeq<_First, _Rem...>, _Sum>
  : intseq_psum<_IntSeq<_Cur..., _Sum>, _IntSeq<_Rem...>, _Sum + _First>
{};

/**
 * @brief prefix-sum an intseq
 */
template<class _Seq>
using intseq_psum_t = typename intseq_psum<intseq<>, _Seq, 0>::type;

/**
 * @brief extract element of typelist
 */
template<std::size_t Idx, class ... T>
struct bundle_element;

// recursive case
template<std::size_t Idx, class Head, class... Tail>
struct bundle_element<Idx, Head, Tail...>
    : public bundle_element<Idx-1, Tail...> { };

// base case
template<class Head, class... Tail>
struct bundle_element<0, Head, Tail...>
{
  using type = Head;
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_TRAITS_H_ */
