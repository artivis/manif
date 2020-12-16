#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>

#include "manif/SE2.h"

#include "bindings_optional.h"
#include "bindings_lie_group_base.h"
#include "bindings_tangent_base.h"

void wrap_SE2(pybind11::module &m)
{
  using Scalar = manif::SE2d::Scalar;
  using Translation = manif::SE2d::Translation;

  pybind11::class_<manif::LieGroupBase<manif::SE2d>, std::unique_ptr<manif::LieGroupBase<manif::SE2d>, py::nodelete>> SE2_base(m, "SE2Base");
  pybind11::class_<manif::TangentBase<manif::SE2Tangentd>, std::unique_ptr<manif::TangentBase<manif::SE2Tangentd>, py::nodelete>> SE2_tan_base(m, "SE2TangentBase");

  pybind11::class_<manif::SE2d, manif::LieGroupBase<manif::SE2d>> SE2(m, "SE2");
  wrap_lie_group_base<manif::SE2d, manif::LieGroupBase<manif::SE2d>>(SE2);

  SE2.def(py::init<const Scalar, const Scalar, const Scalar>());
  SE2.def(py::init<const Scalar, const Scalar, const Scalar, const Scalar>());
  SE2.def(py::init<const Scalar, const Scalar, const std::complex<Scalar>&>());
  SE2.def(py::init<const Translation&, const std::complex<Scalar>&>());
  SE2.def(py::init<const Eigen::Transform<Scalar, 2, Eigen::Isometry>&>());

  SE2.def("transform", &manif::SE2d::transform);
  // SE2.def("isometry", &manif::SE2d::isometry);
  SE2.def("rotation", &manif::SE2d::rotation);
  SE2.def("translation", &manif::SE2d::translation);
  SE2.def("real", &manif::SE2d::real);
  SE2.def("imag", &manif::SE2d::imag);
  SE2.def("angle", &manif::SE2d::angle);
  SE2.def("x", &manif::SE2d::x);
  SE2.def("y", &manif::SE2d::y);
  SE2.def("normalize", &manif::SE2d::normalize);

  pybind11::class_<manif::SE2Tangentd, manif::TangentBase<manif::SE2Tangentd>> SE2_tan(m, "SE2Tangent");
  wrap_tangent_base<manif::SE2Tangentd, manif::TangentBase<manif::SE2Tangentd>>(SE2_tan);

  SE2_tan.def(py::init<const Scalar, const Scalar, const Scalar>());

  SE2_tan.def("x", &manif::SE2Tangentd::x);
  SE2_tan.def("y", &manif::SE2Tangentd::y);
  SE2_tan.def("angle", &manif::SE2Tangentd::angle);
}
