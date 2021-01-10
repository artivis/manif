#ifndef _MANIF_PYTHON_BINDINGS_TANGENT_BASE_H_
#define _MANIF_PYTHON_BINDINGS_TANGENT_BASE_H_

namespace py = pybind11;

template <typename _Tangent, typename... _Args>
void wrap_tangent_base(py::class_<_Tangent, _Args...>& py_class) {

  using Scalar = typename _Tangent::Scalar;
  using LieGroup = typename _Tangent::LieGroup;
  using DataType = typename _Tangent::DataType;
  using Jacobian = typename _Tangent::Jacobian;
  using OptJacobianRef = typename _Tangent::OptJacobianRef;
  using InnerWeight = typename _Tangent::InnerWeight;

  py_class.attr("Dim") = _Tangent::Dim;
  py_class.attr("DoF") = _Tangent::DoF;
  py_class.attr("RepSize") = _Tangent::RepSize;

  py_class.def(py::init<>());
  py_class.def(py::init<const DataType&>());

  py_class.def(
    "coeffs_copy",
    static_cast<DataType& (_Tangent::*)(void)>(&_Tangent::coeffs)
  ); // Makes a copy!
  py_class.def(
    "coeffs",
    static_cast<DataType& (_Tangent::*)(void)>(&_Tangent::coeffs),
    py::return_value_policy::reference_internal
  );

  py_class.def("generator", &_Tangent::generator);

  // py_class.def("innerWeights", &_Tangent::w);
  py_class.def("inner", &_Tangent::template inner<_Tangent>);

  py_class.def("weightedNorm", &_Tangent::weightedNorm);
  py_class.def("squaredWeightedNorm", &_Tangent::squaredWeightedNorm);
  py_class.def("hat", &_Tangent::hat);

  py_class.def(
    "exp",
    &_Tangent::exp,
    py::arg_v("J_out_self", OptJacobianRef(), "None")
  );

  py_class.def(
    "rplus",
    &_Tangent::rplus,
    py::arg("state"),
    py::arg_v("J_out_self", OptJacobianRef(), "None"),
    py::arg_v("J_out_state", OptJacobianRef(), "None")
  );

  py_class.def(
    "lplus",
    &_Tangent::lplus,
    py::arg("state"),
    py::arg_v("J_out_self", OptJacobianRef(), "None"),
    py::arg_v("J_out_state", OptJacobianRef(), "None")
  );

  py_class.def(
    "plus",
    static_cast<LieGroup (_Tangent::*)(const LieGroup&, OptJacobianRef, OptJacobianRef) const>(&_Tangent::plus),
    py::arg("state"),
    py::arg_v("J_out_self", OptJacobianRef(), "None"),
    py::arg_v("J_out_state", OptJacobianRef(), "None")
  );

  py_class.def(
    "plus",
    &_Tangent::template plus<_Tangent>,
    py::arg("other"),
    py::arg_v("J_out_self", OptJacobianRef(), "None"),
    py::arg_v("J_out_other", OptJacobianRef(), "None")
  );

  py_class.def(
    "minus",
    &_Tangent::template minus<_Tangent>,
    py::arg("other"),
    py::arg_v("J_out_self", OptJacobianRef(), "None"),
    py::arg_v("J_out_other", OptJacobianRef(), "None")
  );

  py_class.def("rjac", &_Tangent::rjac);
  py_class.def("ljac", &_Tangent::ljac);
  // py_class.def("rjacinv", &_Tangent::rjacinv);
  // py_class.def("ljacinv", &_Tangent::ljacinv);

  py_class.def("smallAdj", &_Tangent::smallAdj);

  py_class.def(
    "isApprox",
    [](const _Tangent& self, const _Tangent& t, Scalar eps) {
      return self.isApprox(t, eps);
    },
    py::arg("other"),
    py::arg_v("eps", Scalar(manif::Constants<Scalar>::eps), "eps")
  );

  py_class.def(
    "isApprox",
    [](const _Tangent& self, const DataType& t, Scalar eps) {
      return self.isApprox(t, eps);
    },
    py::arg("other"),
    py::arg_v("eps", Scalar(manif::Constants<Scalar>::eps), "eps")
  );

  py_class.def("setZero", &_Tangent::setZero);
  py_class.def("setRandom", &_Tangent::setRandom);

  py_class.def_static("Zero", &_Tangent::Zero);
  py_class.def_static("Random", &_Tangent::Random);
  py_class.def_static("Generator", &_Tangent::Generator);
  py_class.def_static("InnerWeights", &_Tangent::W);

  // operator overloads
  py_class.def(-py::self)
          .def(py::self + LieGroup())
          .def(py::self + py::self)
          // .def(py::self += py::self)
          .def(py::self - py::self)
          // .def(py::self -= py::self)
          .def(py::self * Scalar())
          .def(Scalar() * py::self)
          .def(py::self / Scalar())
          .def(py::self == py::self)
          ;

  // Jacobian() @ py::self
  py_class.def("__rmatmul__", [](const _Tangent& t, py::array_t<Scalar> lhs) {

    py::buffer_info lhs_buf = lhs.request();

    if (lhs_buf.ndim != 2)
        throw std::runtime_error("Number of dimensions must be 2");

    if (lhs_buf.size != _Tangent::DoF * _Tangent::DoF)
        throw std::runtime_error("Input shapes must match");

    _Tangent result = Eigen::Map<Jacobian>(static_cast<Scalar*>(lhs_buf.ptr)) * t;

    return result;

  }, py::is_operator());

  // Jacobian() * py::self
  py_class.def("__rmul__", [](const _Tangent& t, py::array_t<Scalar> lhs) {

    py::buffer_info lhs_buf = lhs.request();

    // if (lhs_buf.ndim != 2)
        // throw std::runtime_error("Number of dimensions must be 2");

    if (lhs_buf.size != _Tangent::DoF * _Tangent::DoF)
        throw std::runtime_error("Input shapes must match");

    _Tangent result = Eigen::Map<Jacobian>(static_cast<Scalar*>(lhs_buf.ptr)) * t;

    return result;

  }, py::is_operator());

  // This is necessary to 'override' numpy's ndarray operators
  // with the __*mul*__ operator above
  py_class.attr("__array_priority__") = 10000;

  py_class.def(
    "__str__",
    [](const _Tangent &t) {
      std::ostringstream ss; ss << t;
      return ss.str();
    }
  );
}

#endif // _MANIF_PYTHON_BINDINGS_TANGENT_BASE_H_
