#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "manif/SO3.h"

#include "bindings_optional.h"
#include "bindings_lie_group_base.h"
#include "bindings_tangent_base.h"

namespace py = pybind11;

void wrap_SO3(py::module &m)
{
  using Scalar = manif::SO3d::Scalar;
  using Quaternion = Eigen::Quaternion<Scalar>;

  py::class_<manif::LieGroupBase<manif::SO3d>, std::unique_ptr<manif::LieGroupBase<manif::SO3d>, py::nodelete>> SO3_base(m, "_SO3Base");
  py::class_<manif::TangentBase<manif::SO3Tangentd>, std::unique_ptr<manif::TangentBase<manif::SO3Tangentd>, py::nodelete>> SO3_tan_base(m, "_SO3TangentBase");

  py::class_<manif::SO3d, manif::LieGroupBase<manif::SO3d>> SO3(m, "SO3");
  py::class_<manif::SO3Tangentd, manif::TangentBase<manif::SO3Tangentd>> SO3_tan(m, "SO3Tangent");

  // group

  SO3.def(py::init<const Scalar, const Scalar, const Scalar>());
  SO3.def(py::init<const Scalar, const Scalar, const Scalar, const Scalar>());
  SO3.def(py::init([](const Eigen::Matrix<Scalar, 4, 1>& quat) {
                       if(abs(quat.norm() - Scalar(1)) >= manif::Constants<Scalar>::eps_s) {
                           throw pybind11::value_error("The quaternion is not normalized!");
                       }

                       return manif::SO3d(quat);
                   }),
      py::arg("quaternion"));

  // SO3.def(py::init<const Quaternion&>());
  // SO3.def(py::init<const Eigen::AngleAxis<Scalar>&>());

  wrap_lie_group_base<manif::SO3d, manif::LieGroupBase<manif::SO3d>>(SO3);

  SO3.def("transform", &manif::SO3d::transform);
  SO3.def("rotation", &manif::SO3d::rotation);
  SO3.def("x", &manif::SO3d::x);
  SO3.def("y", &manif::SO3d::y);
  SO3.def("z", &manif::SO3d::z);
  SO3.def("w", &manif::SO3d::w);
  SO3.def(
      "quat",
      [](const manif::SO3d& so3) -> Eigen::Matrix<Scalar, 4, 1> { return so3.coeffs(); });

  SO3.def(
      "quat",
      [](manif::SO3d& so3, const Eigen::Matrix<Scalar, 4, 1>& quaternion) {
          if(abs(quaternion.norm() - Scalar(1)) >= manif::Constants<Scalar>::eps_s) {
              throw pybind11::value_error("The quaternion is not normalized!");
          }
          so3.quat(quaternion);
      },
      py::arg("quaternion"));

  SO3.def("normalize", &manif::SO3d::normalize);

  // tangent

  wrap_tangent_base<manif::SO3Tangentd, manif::TangentBase<manif::SO3Tangentd>>(SO3_tan);

  SO3_tan.def("x", &manif::SO3Tangentd::x);
  SO3_tan.def("y", &manif::SO3Tangentd::y);
  SO3_tan.def("z", &manif::SO3Tangentd::z);
}
