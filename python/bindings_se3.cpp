#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "manif/SE3.h"

#include "bindings_optional.h"
#include "bindings_lie_group_base.h"
#include "bindings_tangent_base.h"

namespace py = pybind11;

void wrap_SE3(py::module &m)
{
  using SE3d = manif::SE3d;
  using Scalar = SE3d::Scalar;
  using Quaternion = Eigen::Quaternion<Scalar>;

  py::class_<manif::LieGroupBase<SE3d>, std::unique_ptr<manif::LieGroupBase<SE3d>, py::nodelete>> SE3_base(m, "_SE3Base");
  py::class_<manif::TangentBase<manif::SE3Tangentd>, std::unique_ptr<manif::TangentBase<manif::SE3Tangentd>, py::nodelete>> SE3_tan_base(m, "_SE3TangentBase");

  py::class_<SE3d, manif::LieGroupBase<SE3d>> SE3(m, "SE3");
  py::class_<manif::SE3Tangentd, manif::TangentBase<manif::SE3Tangentd>> SE3_tan(m, "SE3Tangent");

  // group

  wrap_lie_group_base<SE3d, manif::LieGroupBase<SE3d>>(SE3);

  SE3.def(py::init<const Scalar, const Scalar, const Scalar,
                   const Scalar, const Scalar, const Scalar>());
  SE3.def(py::init([](const SE3d::Translation& pos, const Eigen::Vector4d& quat) {
                       if(abs(quat.norm() - Scalar(1)) >= manif::Constants<Scalar>::eps_s) {
                           throw pybind11::value_error("The quaternion is not normalized!");
                       }
                       return manif::SE3d(pos, quat);
                   }),
      py::arg("position"),
      py::arg("quaternion"));

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

  //tangent

  wrap_tangent_base<manif::SE3Tangentd, manif::TangentBase<manif::SE3Tangentd>>(SE3_tan);

  // SE3_tan.def("v", &manif::SE3Tangentd::v);
  // SE3_tan.def("w", &manif::SE3Tangentd::w);
}
