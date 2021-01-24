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
  using Scalar = manif::SO3d::Scalar;
  using Quaternion = Eigen::Quaternion<Scalar>;

  pybind11::class_<manif::LieGroupBase<manif::SO3d>, std::unique_ptr<manif::LieGroupBase<manif::SO3d>, py::nodelete>> SO3_base(m, "SO3Base");
  pybind11::class_<manif::TangentBase<manif::SO3Tangentd>, std::unique_ptr<manif::TangentBase<manif::SO3Tangentd>, py::nodelete>> SO3_tan_base(m, "SO3TangentBase");

  pybind11::class_<manif::SO3d, manif::LieGroupBase<manif::SO3d>> SO3(m, "SO3");

  SO3.def(py::init<const Scalar, const Scalar, const Scalar>());
  SO3.def(py::init<const Scalar, const Scalar, const Scalar, const Scalar>());
  // SO3.def(py::init<const Quaternion&>());
  // SO3.def(py::init<const Eigen::AngleAxis<Scalar>&>());

  wrap_lie_group_base<manif::SO3d, manif::LieGroupBase<manif::SO3d>>(SO3);

  SO3.def("transform", &manif::SO3d::transform);
  SO3.def("rotation", &manif::SO3d::rotation);
  SO3.def("x", &manif::SO3d::x);
  SO3.def("y", &manif::SO3d::y);
  SO3.def("z", &manif::SO3d::z);
  SO3.def("w", &manif::SO3d::w);
  // SO3.def("quat", &manif::SO3d::quat);
  SO3.def("normalize", &manif::SO3d::normalize);

  pybind11::class_<manif::SO3Tangentd, manif::TangentBase<manif::SO3Tangentd>> SO3_tan(m, "SO3Tangent");
  wrap_tangent_base<manif::SO3Tangentd, manif::TangentBase<manif::SO3Tangentd>>(SO3_tan);

  SO3_tan.def("x", &manif::SO3Tangentd::x);
  SO3_tan.def("y", &manif::SO3Tangentd::y);
  SO3_tan.def("z", &manif::SO3Tangentd::z);
}
