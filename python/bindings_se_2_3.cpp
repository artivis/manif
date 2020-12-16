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
  using Scalar = manif::SE_2_3d::Scalar;
  using Translation = manif::SE_2_3d::Translation;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using LinearVelocity = manif::SE_2_3d::LinearVelocity;

  pybind11::class_<manif::LieGroupBase<manif::SE_2_3d>, std::unique_ptr<manif::LieGroupBase<manif::SE_2_3d>, py::nodelete>> SE_2_3_base(m, "SE_2_3Base");
  pybind11::class_<manif::TangentBase<manif::SE_2_3Tangentd>, std::unique_ptr<manif::TangentBase<manif::SE_2_3Tangentd>, py::nodelete>> SE_2_3_tan_base(m, "SE_2_3TangentBase");

  pybind11::class_<manif::SE_2_3d, manif::LieGroupBase<manif::SE_2_3d>> SE_2_3(m, "SE_2_3");
  wrap_lie_group_base<manif::SE_2_3d, manif::LieGroupBase<manif::SE_2_3d>>(SE_2_3);

  // SE_2_3.def(py::init<const Translation&, const Quaternion&, const LinearVelocity&>());
  // SE_2_3.def(py::init<const Translation&, const Eigen::AngleAxis<Scalar>&, const LinearVelocity&>());
  // SE_2_3.def(py::init<const Translation&, const manif::SO3<Scalar>&, const LinearVelocity&>());
  SE_2_3.def(py::init<const Scalar, const Scalar, const Scalar,
                      const Scalar, const Scalar, const Scalar,
                      const Scalar, const Scalar, const Scalar >());
  // SE_2_3.def(py::init<igen::Transform<Scalar, 3, Eigen::Isometry>&, const LinearVelocity&>());

  // SE_2_3.def("isometry", &manif::SE_2_3d::isometry);
  SE_2_3.def("rotation", &manif::SE_2_3d::rotation);
  SE_2_3.def("quat", &manif::SE_2_3d::quat);
  SE_2_3.def("translation", &manif::SE_2_3d::translation);
  SE_2_3.def("x", &manif::SE_2_3d::x);
  SE_2_3.def("y", &manif::SE_2_3d::y);
  SE_2_3.def("z", &manif::SE_2_3d::z);
  SE_2_3.def("linearVelocity", &manif::SE_2_3d::linearVelocity);
  SE_2_3.def("vx", &manif::SE_2_3d::vx);
  SE_2_3.def("vy", &manif::SE_2_3d::vy);
  SE_2_3.def("vz", &manif::SE_2_3d::vz);
  SE_2_3.def("normalize", &manif::SE_2_3d::normalize);

  pybind11::class_<manif::SE_2_3Tangentd, manif::TangentBase<manif::SE_2_3Tangentd>> SE_2_3_tan(m, "SE_2_3Tangent");
  wrap_tangent_base<manif::SE_2_3Tangentd, manif::TangentBase<manif::SE_2_3Tangentd>>(SE_2_3_tan);

  // SE_2_3_tan.def("v", &manif::SE3Tangentd::v);
  // SE_2_3_tan.def("w", &manif::SE3Tangentd::w);
  // SE_2_3_tan.def("a", &manif::SE3Tangentd::a);
}
