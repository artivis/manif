#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "manif/SE_2_3.h"

#include "bindings_optional.h"
#include "bindings_lie_group_base.h"
#include "bindings_tangent_base.h"

void wrap_SE_2_3(pybind11::module &m)
{
  pybind11::class_<manif::LieGroupBase<manif::SE_2_3d>, std::unique_ptr<manif::LieGroupBase<manif::SE_2_3d>, py::nodelete>> SE_2_3_base(m, "SE_2_3Base");
  pybind11::class_<manif::TangentBase<manif::SE_2_3Tangentd>, std::unique_ptr<manif::TangentBase<manif::SE_2_3Tangentd>, py::nodelete>> SE_2_3_tan_base(m, "SE_2_3TangentBase");

  pybind11::class_<manif::SE_2_3d, manif::LieGroupBase<manif::SE_2_3d>> SE_2_3(m, "SE_2_3");
  pybind11::class_<manif::SE_2_3Tangentd, manif::TangentBase<manif::SE_2_3Tangentd>> SE_2_3_tan(m, "SE_2_3Tangent");

  wrap_lie_group_base<manif::SE_2_3d, manif::LieGroupBase<manif::SE_2_3d>>(SE_2_3);
  wrap_tangent_base<manif::SE_2_3Tangentd, manif::TangentBase<manif::SE_2_3Tangentd>>(SE_2_3_tan);
}
