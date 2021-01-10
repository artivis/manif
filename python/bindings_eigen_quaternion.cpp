#include <Eigen/Geometry>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>

void wrap_Eigen_quaternion(pybind11::module &m)
{
  using Scalar = double;
  using Quaternion = Eigen::Quaternion<Scalar>;

  pybind11::class_<Eigen::QuaternionBase<Quaternion>> quat_base(m, "QuaternionBase");
  pybind11::class_<Quaternion, Eigen::QuaternionBase<Quaternion>> quat(m, "Quaternion");

  quat.attr("__doc__") = "Python bindings for Eigen::Quaternion<double>.";

  quat.def(pybind11::init<>())
      .def_static("Identity", &Quaternion::Identity)
#if EIGEN_VERSION_AT_LEAST(3,3,0)
      .def_static("UnitRandom", &Quaternion::UnitRandom)
#endif
      .def(pybind11::init<const Eigen::Vector4d&>())
      .def(pybind11::init<const Eigen::Matrix3d&>())
      .def(pybind11::init<const Eigen::AngleAxisd&>())
      .def(pybind11::init<const Scalar&, const Scalar&, const Scalar&, const Scalar&>())
      // .def(pybind11::init([](const Class& other) {
      //   return other;
      // }), pybind11::arg("other"))

      // .def(
      //   "coeffs",
      //   static_cast<const Eigen::internal::traits<Quaternion>::Coefficients& (Quaternion::*)(void)>(&Quaternion::coeffs),
      //   pybind11::return_value_policy::reference_internal
      // )

      .def("angularDistance", &Quaternion::template angularDistance<Quaternion>)
      .def("conjugate", &Quaternion::conjugate)
      .def("dot", &Quaternion::template dot<Quaternion>)
      .def("inverse", &Quaternion::inverse)
      .def(
        "isApprox",
        &Quaternion::template isApprox<Quaternion>,
        pybind11::arg("other"),
        pybind11::arg_v("prec", Eigen::NumTraits<Scalar>::dummy_precision(), "Precision")
      )
      .def("norm", &Quaternion::norm)
      .def("normalize", &Quaternion::normalize)
      .def("normalized", &Quaternion::normalized)
      .def("setIdentity", &Quaternion::setIdentity)
      .def("slerp", &Quaternion::template slerp<Quaternion>)
      .def("squaredNorm", &Quaternion::squaredNorm)
      .def("toRotationMatrix", &Quaternion::toRotationMatrix)
      // .def("setFromTwoVectors", &Quaternion::setFromTwoVectors)

      .def("matrix", &Quaternion::matrix)

      .def(
        "vec",
        [](const Quaternion& self) { return self.vec(); },
        pybind11::return_value_policy::reference_internal
      )

      .def("x", static_cast<double &(Quaternion::*)()>(&Quaternion::x))
      .def("y", static_cast<double &(Quaternion::*)()>(&Quaternion::y))
      .def("z", static_cast<double &(Quaternion::*)()>(&Quaternion::z))
      .def("w", static_cast<double &(Quaternion::*)()>(&Quaternion::w))

      .def(pybind11::self * pybind11::self)
      // .def(pybind11::self = Eigen::AngleAxisd())
      // .def(pybind11::self = Eigen::Matrix)

      .def(
        "__str__",
        [](const Quaternion &q) {
          std::ostringstream ss; ss << q.coeffs();
          return ss.str();
        }
      );
}
