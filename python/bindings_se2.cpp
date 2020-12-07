#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "manif/SE2.h"

#include "bindings_optional.h"
#include "bindings_lie_group_base.h"
#include "bindings_tangent_base.h"

void wrap_SE2(pybind11::module &m)
{
  pybind11::class_<manif::LieGroupBase<manif::SE2d>, std::unique_ptr<manif::LieGroupBase<manif::SE2d>, py::nodelete>> SE2_base(m, "SE2Base");
  pybind11::class_<manif::TangentBase<manif::SE2Tangentd>, std::unique_ptr<manif::TangentBase<manif::SE2Tangentd>, py::nodelete>> SE2_tan_base(m, "SE2TangentBase");

  pybind11::class_<manif::SE2d, manif::LieGroupBase<manif::SE2d>> SE2(m, "SE2");
  pybind11::class_<manif::SE2Tangentd, manif::TangentBase<manif::SE2Tangentd>> SE2_tan(m, "SE2Tangent");

  wrap_lie_group_base<manif::SE2d, manif::LieGroupBase<manif::SE2d>>(SE2);
  wrap_tangent_base<manif::SE2Tangentd, manif::TangentBase<manif::SE2Tangentd>>(SE2_tan);
}
