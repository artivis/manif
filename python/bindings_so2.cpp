#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "manif/SO2.h"

#include "bindings_optional.h"
#include "bindings_lie_group_base.h"
#include "bindings_tangent_base.h"

namespace py = pybind11;

void wrap_SO2(py::module &m)
{
  using Scalar = manif::SO2d::Scalar;

  py::class_<manif::LieGroupBase<manif::SO2d>, std::unique_ptr<manif::LieGroupBase<manif::SO2d>, py::nodelete>> SO2_base(m, "_SO2Base");
  py::class_<manif::TangentBase<manif::SO2Tangentd>, std::unique_ptr<manif::TangentBase<manif::SO2Tangentd>, py::nodelete>> SO2_tan_base(m, "_SO2TangentBase");

  py::class_<manif::SO2d, manif::LieGroupBase<manif::SO2d>> SO2(m, "SO2", "The SO2 class");
  py::class_<manif::SO2Tangentd, manif::TangentBase<manif::SO2Tangentd>> SO2_tan(m, "SO2Tangent");

  //group

  wrap_lie_group_base<manif::SO2d, manif::LieGroupBase<manif::SO2d>>(SO2);

  SO2.def(py::init<const Scalar>());
  SO2.def(py::init<const Scalar, const Scalar>());

  SO2.def("transform", &manif::SO2d::transform, "Get the transformation matrix");
  SO2.def("rotation", &manif::SO2d::rotation, "Get the rotation matrix");
  SO2.def("real", &manif::SO2d::real);
  SO2.def("imag", &manif::SO2d::imag);
  SO2.def("angle", &manif::SO2d::angle);
  SO2.def("normalize", &manif::SO2d::normalize);

  // tangent

  wrap_tangent_base<manif::SO2Tangentd, manif::TangentBase<manif::SO2Tangentd>>(SO2_tan);

  SO2_tan.def(py::init<const Scalar>());
  SO2_tan.def("angle", &manif::SO2Tangentd::angle);
}
