#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "manif/SE3.h"

#include "bindings_optional.h"
#include "bindings_lie_group_base.h"
#include "bindings_tangent_base.h"

void wrap_SE3(pybind11::module &m)
{
  using SE3d = manif::SE3d;
  using Scalar = SE3d::Scalar;
  using Quaternion = Eigen::Quaternion<Scalar>;

  pybind11::class_<manif::LieGroupBase<SE3d>, std::unique_ptr<manif::LieGroupBase<SE3d>, py::nodelete>> SE3_base(m, "SE3Base");
  pybind11::class_<manif::TangentBase<manif::SE3Tangentd>, std::unique_ptr<manif::TangentBase<manif::SE3Tangentd>, py::nodelete>> SE3_tan_base(m, "SE3TangentBase");

  pybind11::class_<SE3d, manif::LieGroupBase<SE3d>> SE3(m, "SE3");
  wrap_lie_group_base<SE3d, manif::LieGroupBase<SE3d>>(SE3);

  SE3.def(py::init<const Scalar, const Scalar, const Scalar,
                   const Scalar, const Scalar, const Scalar>());
  // SE3.def(py::init<const Translation&, const Quaternion&>());
  // SE3.def(py::init<const Translation&, const Eigen::AngleAxis<Scalar>&>());
  // SE3.def(py::init<const Translation&, const manif::SO3<Scalar>&>());
  // SE3.def(py::init<igen::Transform<Scalar, 3, Eigen::Isometry>&>());

  SE3.def("transform", &SE3d::transform);
  // SE3.def("isometry", &SE3d::isometry);
  SE3.def("rotation", &SE3d::rotation);
  // SE3.def("quat", &SE3d::quat);
  SE3.def(
    "translation",
    static_cast<SE3d::Translation (SE3d::*)(void) const>(&SE3d::translation)
  );
  SE3.def("x", &SE3d::x);
  SE3.def("y", &SE3d::y);
  SE3.def("z", &SE3d::z);
  SE3.def("normalize", &SE3d::normalize);

  pybind11::class_<manif::SE3Tangentd, manif::TangentBase<manif::SE3Tangentd>> SE3_tan(m, "SE3Tangent");
  wrap_tangent_base<manif::SE3Tangentd, manif::TangentBase<manif::SE3Tangentd>>(SE3_tan);

  // SE3_tan.def("v", &manif::SE3Tangentd::v);
  // SE3_tan.def("w", &manif::SE3Tangentd::w);
}
