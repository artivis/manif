#ifndef _MANIF_PYTHON_BINDINGS_OPTIONAL_H_
#define _MANIF_PYTHON_BINDINGS_OPTIONAL_H_

#include <tl/optional.hpp>

namespace pybind11 { namespace detail {
    template <typename T>
    struct type_caster<tl::optional<T>> : optional_caster<tl::optional<T>> {};
}}

#endif // _MANIF_PYTHON_BINDINGS_OPTIONAL_H_
