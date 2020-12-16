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
  using Scalar = manif::SO2d::Scalar;

  pybind11::class_<manif::LieGroupBase<manif::SO2d>, std::unique_ptr<manif::LieGroupBase<manif::SO2d>, py::nodelete>> SO2_base(m, "SO2Base");
  pybind11::class_<manif::TangentBase<manif::SO2Tangentd>, std::unique_ptr<manif::TangentBase<manif::SO2Tangentd>, py::nodelete>> SO2_tan_base(m, "SO2TangentBase");

  pybind11::class_<manif::SO2d, manif::LieGroupBase<manif::SO2d>> SO2(m, "SO2");
  wrap_lie_group_base<manif::SO2d, manif::LieGroupBase<manif::SO2d>>(SO2);

  SO2.def(py::init<const Scalar>());
  SO2.def(py::init<const Scalar, const Scalar>());

  SO2.def("transform", &manif::SO2d::transform);
  SO2.def("rotation", &manif::SO2d::rotation);
  SO2.def("real", &manif::SO2d::real);
  SO2.def("imag", &manif::SO2d::imag);
  SO2.def("angle", &manif::SO2d::angle);
  SO2.def("normalize", &manif::SO2d::normalize);

  pybind11::class_<manif::SO2Tangentd, manif::TangentBase<manif::SO2Tangentd>> SO2_tan(m, "SO2Tangent");
  wrap_tangent_base<manif::SO2Tangentd, manif::TangentBase<manif::SO2Tangentd>>(SO2_tan);

  SO2_tan.def(py::init<const Scalar>());

  SO2_tan.def("angle", &manif::SO2Tangentd::angle);
}
