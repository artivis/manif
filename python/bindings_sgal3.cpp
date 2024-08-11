#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "manif/SGal3.h"

#include "bindings_optional.h"
#include "bindings_lie_group_base.h"
#include "bindings_tangent_base.h"

namespace py = pybind11;

void wrap_SGal3(py::module &m) {
  using Scalar = manif::SGal3d::Scalar;
  using Translation = manif::SGal3d::Translation;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using LinearVelocity = manif::SGal3d::LinearVelocity;

  py::class_<
    manif::LieGroupBase<manif::SGal3d>,
    std::unique_ptr<manif::LieGroupBase<manif::SGal3d>, py::nodelete>
  > SGal3_base(m, "_SGal3Base");
  py::class_<
    manif::TangentBase<manif::SGal3Tangentd>,
    std::unique_ptr<manif::TangentBase<manif::SGal3Tangentd>, py::nodelete>
  > SGal3_tan_base(m, "_SGal3TangentBase");

  py::class_<manif::SGal3d, manif::LieGroupBase<manif::SGal3d>> SGal3(m, "SGal3");
  py::class_<
    manif::SGal3Tangentd, manif::TangentBase<manif::SGal3Tangentd>
  > SGal3_tan(m, "SGal3Tangent");

  //group

  wrap_lie_group_base<manif::SGal3d, manif::LieGroupBase<manif::SGal3d>>(SGal3);

  // SGal3.def(py::init<const Translation&, const Quaternion&, const LinearVelocity&>());
  // SGal3.def(py::init<const Translation&, const Eigen::AngleAxis<Scalar>&, const LinearVelocity&>());
  // SGal3.def(py::init<const Translation&, const manif::SO3<Scalar>&, const LinearVelocity&>());
  SGal3.def(
    py::init<
      const Scalar, const Scalar, const Scalar,
      const Scalar, const Scalar, const Scalar,
      const Scalar, const Scalar, const Scalar,
      const Scalar
    >()
  );
  // SGal3.def(py::init<igen::Transform<Scalar, 3, Eigen::Isometry>&, const LinearVelocity&>());

  // SGal3.def("isometry", &manif::SGal3d::isometry);
  SGal3.def("rotation", &manif::SGal3d::rotation);
  // SGal3.def("quat", &manif::SGal3d::quat);
  SGal3.def("translation", &manif::SGal3d::translation);
  SGal3.def("x", &manif::SGal3d::x);
  SGal3.def("y", &manif::SGal3d::y);
  SGal3.def("z", &manif::SGal3d::z);
  SGal3.def("linearVelocity", &manif::SGal3d::linearVelocity);
  SGal3.def("vx", &manif::SGal3d::vx);
  SGal3.def("vy", &manif::SGal3d::vy);
  SGal3.def("vz", &manif::SGal3d::vz);
  SGal3.def("t", &manif::SGal3d::t);
  SGal3.def("normalize", &manif::SGal3d::normalize);

  // tangent
  wrap_tangent_base<
    manif::SGal3Tangentd, manif::TangentBase<manif::SGal3Tangentd>
  >(SGal3_tan);

  // SGal3_tan.def("v", &manif::SE3Tangentd::v);
  // SGal3_tan.def("w", &manif::SE3Tangentd::w);
  // SGal3_tan.def("a", &manif::SE3Tangentd::a);
}
