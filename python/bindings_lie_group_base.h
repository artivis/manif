#ifndef _MANIF_PYTHON_BINDINGS_LIE_GROUP_BASE_H_
#define _MANIF_PYTHON_BINDINGS_LIE_GROUP_BASE_H_

namespace py = pybind11;

template <typename _LieGroup, typename... _Args>
void wrap_lie_group_base(py::class_<_LieGroup, _Args...>& py_class) {

  using Scalar = typename _LieGroup::Scalar;
  using Tangent = typename _LieGroup::Tangent;
  using Vector = typename _LieGroup::Vector;
  using DataType = typename _LieGroup::DataType;
  using OptJacobianRef = typename _LieGroup::OptJacobianRef;

  py_class.attr("Dim") = _LieGroup::Dim;
  py_class.attr("DoF") = _LieGroup::DoF;
  py_class.attr("RepSize") = _LieGroup::RepSize;

  py_class.def(py::init<>());
  py_class.def(py::init<const DataType&>());

  py_class.def(
    "coeffs_copy",
    static_cast<DataType& (_LieGroup::*)(void)>(&_LieGroup::coeffs)
  ); // Makes a copy!

  py_class.def(
    "coeffs",
    static_cast<DataType& (_LieGroup::*)(void)>(&_LieGroup::coeffs),
    py::return_value_policy::reference_internal
  );

  py_class.def(
    "inverse",
    &_LieGroup::inverse,
    py::arg_v("J_m_t", OptJacobianRef(), "None")
  );

  py_class.def(
    "log",
    &_LieGroup::log,
    py::arg_v("J_m_t", OptJacobianRef(), "None")
  );

  py_class.def("adj", &_LieGroup::adj);

  py_class.def(
    "compose", &_LieGroup::template compose<_LieGroup>,
    py::arg("other"),
    py::arg_v("J_mc_ma", OptJacobianRef(), "None"),
    py::arg_v("J_mc_mb", OptJacobianRef(), "None")
  );

  py_class.def(
    "between", &_LieGroup::template between<_LieGroup>,
    py::arg("other"),
    py::arg_v("J_mc_ma", OptJacobianRef(), "None"),
    py::arg_v("J_mc_mb", OptJacobianRef(), "None")
  );

  // py_class.def("act", &_LieGroup::template act<Vector>,
  //   py::arg("v"),
  //   py::arg_v("J_vout_m", tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, _LieGroup::Dim, _LieGroup::DoF>>>(), "None"),
  //   py::arg_v("J_vout_v", tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, _LieGroup::Dim, _LieGroup::Dim>>>(), "None")
  // );
  py_class.def("act", [](const _LieGroup& self, const Vector& v) {
    return self.act(v);
  });

  py_class.def(
    "isApprox",
    &_LieGroup::template isApprox<_LieGroup>,
    py::arg("other"),
    py::arg_v("eps", Scalar(manif::Constants<Scalar>::eps), "eps")
  );

  py_class.def(
    "rplus",
    &_LieGroup::template rplus<Tangent>,
    py::arg("t"),
    py::arg_v("J_mout_m", OptJacobianRef(), "None"),
    py::arg_v("J_mout_t", OptJacobianRef(), "None")
  );

  py_class.def(
    "lplus",
    &_LieGroup::template lplus<Tangent>,
    py::arg("t"),
    py::arg_v("J_mout_m", OptJacobianRef(), "None"),
    py::arg_v("J_mout_t", OptJacobianRef(), "None")
  );

  py_class.def(
    "plus",
    &_LieGroup::template plus<Tangent>,
    py::arg("t"),
    py::arg_v("J_mout_m", OptJacobianRef(), "None"),
    py::arg_v("J_mout_t", OptJacobianRef(), "None")
  );

  py_class.def(
    "rminus",
    &_LieGroup::template rminus<_LieGroup>,
    py::arg("t"),
    py::arg_v("J_t_ma", OptJacobianRef(), "None"),
    py::arg_v("J_t_mb", OptJacobianRef(), "None")
  );

  py_class.def(
    "lminus",
    &_LieGroup::template lminus<_LieGroup>,
    py::arg("t"),
    py::arg_v("J_t_ma", OptJacobianRef(), "None"),
    py::arg_v("J_t_mb", OptJacobianRef(), "None")
  );

  py_class.def(
    "minus",
    &_LieGroup::template minus<_LieGroup>,
    py::arg("t"),
    py::arg_v("J_t_ma", OptJacobianRef(), "None"),
    py::arg_v("J_t_mb", OptJacobianRef(), "None")
  );

  py_class.def("setIdentity", &_LieGroup::setIdentity);
  py_class.def("setRandom", &_LieGroup::setRandom);

  py_class.def_static("Identity", &_LieGroup::Identity);
  py_class.def_static("Random", &_LieGroup::Random);

  py_class.def(py::self + Tangent())
          // .def(py::self += Tangent())
          .def(py::self - py::self)
          .def(py::self * py::self)
          // .def(py::self *= py::self)
          .def(py::self == py::self)
          ;

  py_class.def(
    "__str__",
    [](const _LieGroup &o) {
      std::ostringstream ss; ss << o;
      return ss.str();
    }
  );
}

#endif // _MANIF_PYTHON_BINDINGS_LIE_GROUP_BASE_H_
