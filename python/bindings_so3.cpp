#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "manif/SO3.h"

#include "bindings_optional.h"
#include "bindings_lie_group_base.h"
#include "bindings_tangent_base.h"

void wrap_SO3(pybind11::module &m)
{
  pybind11::class_<manif::LieGroupBase<manif::SO3d>, std::unique_ptr<manif::LieGroupBase<manif::SO3d>, py::nodelete>> SO3_base(m, "SO3Base");
  pybind11::class_<manif::TangentBase<manif::SO3Tangentd>, std::unique_ptr<manif::TangentBase<manif::SO3Tangentd>, py::nodelete>> SO3_tan_base(m, "SO3TangentBase");

  pybind11::class_<manif::SO3d, manif::LieGroupBase<manif::SO3d>> SO3(m, "SO3");
  pybind11::class_<manif::SO3Tangentd, manif::TangentBase<manif::SO3Tangentd>> SO3_tan(m, "SO3Tangent");

  wrap_lie_group_base<manif::SO3d, manif::LieGroupBase<manif::SO3d>>(SO3);
  wrap_tangent_base<manif::SO3Tangentd, manif::TangentBase<manif::SO3Tangentd>>(SO3_tan);
}
