#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "manif/SO2.h"

#include "bindings_optional.h"
#include "bindings_lie_group_base.h"
#include "bindings_tangent_base.h"

void wrap_SO2(pybind11::module &m)
{
  pybind11::class_<manif::LieGroupBase<manif::SO2d>, std::unique_ptr<manif::LieGroupBase<manif::SO2d>, py::nodelete>> SO2_base(m, "SO2Base");
  pybind11::class_<manif::TangentBase<manif::SO2Tangentd>, std::unique_ptr<manif::TangentBase<manif::SO2Tangentd>, py::nodelete>> SO2_tan_base(m, "SO2TangentBase");

  pybind11::class_<manif::SO2d, manif::LieGroupBase<manif::SO2d>> SO2(m, "SO2");
  pybind11::class_<manif::SO2Tangentd, manif::TangentBase<manif::SO2Tangentd>> SO2_tan(m, "SO2Tangent");

  wrap_lie_group_base<manif::SO2d, manif::LieGroupBase<manif::SO2d>>(SO2);
  wrap_tangent_base<manif::SO2Tangentd, manif::TangentBase<manif::SO2Tangentd>>(SO2_tan);
}
