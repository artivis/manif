#ifndef _MANIF_MANIF_IMPL_CORE_UNARY_STORAGE_H_
#define _MANIF_MANIF_IMPL_CORE_UNARY_STORAGE_H_

namespace manif {
namespace internal {

template <typename T>
struct storage_selector {
    using type = T;
};

template <typename T>
struct storage_selector<T&> {
    using type = T&;
};

template <typename T>
struct storage_selector<const T&> {
    using type = const T&;
};

template <typename T>
struct storage_selector<T&&> {
    using type = T;
};

template <typename T>
using storage_t = typename storage_selector<T>::type;

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_IMPL_CORE_UNARY_STORAGE_H_
