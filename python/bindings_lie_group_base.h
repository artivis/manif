#ifndef _MANIF_PYTHON_BINDINGS_LIE_GROUP_BASE_H_
#define _MANIF_PYTHON_BINDINGS_LIE_GROUP_BASE_H_

template <typename _LieGroup, typename... _Args>
void wrap_lie_group_base(pybind11::class_<_LieGroup, _Args...>& py_class) {

  using Scalar = typename _LieGroup::Scalar;
  using Tangent = typename _LieGroup::Tangent;
  using Vector = typename _LieGroup::Vector;
  using DataType = typename _LieGroup::DataType;
  using OptJacobianRef = typename _LieGroup::OptJacobianRef;

  py_class.attr("Dim") = _LieGroup::Dim;
  py_class.attr("DoF") = _LieGroup::DoF;
  py_class.attr("RepSize") = _LieGroup::RepSize;

  py_class.def(
    pybind11::init<>(),
    "Default constructor, uninitialized data."
  );

  py_class.def(
    pybind11::init<const DataType&>(),
    "Constructor given data vector."
  );

  py_class.def(
    "coeffs_copy",
    static_cast<DataType& (_LieGroup::*)(void)>(&_LieGroup::coeffs),
    "Return a copy of underlying data."
  ); // Makes a copy!

  py_class.def(
    "coeffs",
    static_cast<DataType& (_LieGroup::*)(void)>(&_LieGroup::coeffs),
    pybind11::return_value_policy::reference_internal,
    "Get a reference to underlying data."
  );

  py_class.def(
    "inverse",
    &_LieGroup::inverse,
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    R"(
      Return the inverse of the Lie group object.

      See Eq. (3).

      Parameters
      ----------
      J_out_self [out] : numpy.ndarray
        Jacobian of the inverse wrt self.
    )"
  );

  py_class.def(
    "log",
    &_LieGroup::log,
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    R"(
      Return the corresponding Lie algebra element in vector form.

      Eq. (24).

      Parameters
      ----------
      J_out_self [out] : numpy.ndarray
        Jacobian of the log wrt self.
    )"
  );

  py_class.def(
    "adj",
    &_LieGroup::adj,
    R"(
      Return the Adjoint of the Lie group object self.

      See Eq. (29).
    )"
  );

  py_class.def(
    "compose",
    &_LieGroup::template compose<_LieGroup>,
    pybind11::arg("other"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_out_other", OptJacobianRef(), "None"),
    R"(
      Return the composition of self and another object of the same Lie group.

      See Eqs. (1,2,3,4).

      Parameters
      ----------
      other : Lie group
        Another object of the same Lie group.
      J_out_self [out] : numpy.ndarray
        Jacobian of the composition wrt self.
      J_out_other [out] : numpy.ndarray
        Jacobian of the composition wrt other.
    )"
  );

  py_class.def(
    "between",
    &_LieGroup::template between<_LieGroup>,
    pybind11::arg("other"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_out_other", OptJacobianRef(), "None"),
    R"(
      Return the between of self and another object of the same Lie group.

      Parameters
      ----------
      other : Lie group
        Another object of the same Lie group.
      J_out_self [out] : numpy.ndarray
        Jacobian of the composition wrt self.
      J_out_other [out] : numpy.ndarray
        Jacobian of the composition wrt other.
    )"
  );

  // That pops some nasty compilation errors.
  // py_class.def(
  //   "act",
  //   &_LieGroup::template act<Vector>,
  //   pybind11::arg("v"),
  //   pybind11::arg_v(
  //     "J_vout_m",
  //     tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, _LieGroup::Dim, _LieGroup::DoF>>>(),
  //     "None"
  //   ),
  //   pybind11::arg_v(
  //     "J_vout_v",
  //     tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, _LieGroup::Dim, _LieGroup::Dim>>>(),
  //     "None"
  //   )
  // );

  py_class.def(
    "act",
    [](
      const _LieGroup& self,
      const Vector& v,
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, _LieGroup::Dim, _LieGroup::DoF>>> Ja,
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, _LieGroup::Dim, _LieGroup::Dim>>> Jb) {
        return self.act(v, Ja, Jb);
    },
    pybind11::arg("p"),
    pybind11::arg_v(
      "J_out_self",
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, _LieGroup::Dim, _LieGroup::DoF>>>(),
      "None"
    ),
    pybind11::arg_v(
      "J_out_p",
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, _LieGroup::Dim, _LieGroup::Dim>>>(),
      "None"
     ),
    R"(
      Get the action of the Lie group object on a point.

      Parameters
      ----------
      p : numpy.array
        A point.
      J_out_self [out] : numpy.ndarray
        Jacobian of the new object wrt self.
      J_out_p [out] : numpy.ndarray
        Jacobian of the new object wrt input point.
    )"
  );

  py_class.def(
    "isApprox",
    &_LieGroup::template isApprox<_LieGroup>,
    pybind11::arg("other"),
    pybind11::arg_v("eps", Scalar(manif::Constants<Scalar>::eps), "1e-10"),
    R"(
      Evaluate whether self and other are 'close'.

      Parameters
      ----------
      other : Lie group
        Another object of the same Lie group.
      eps : double
        Threshold for equality comparison. Default: 1e-10.
    )"
  );

  py_class.def(
    "rplus",
    &_LieGroup::template rplus<Tangent>,
    pybind11::arg("tau"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_out_tau", OptJacobianRef(), "None" ),
    R"(
      Right oplus operation of the Lie group.

      See Eq. (25).

      Parameters
      ----------
      tau : Lie group tangent
        An element of the tangent of the Lie group.
      J_out_self [out] : numpy.ndarray
        Jacobian of the oplus operation wrt self.
      J_out_tau [out] : numpy.ndarray
        Jacobian of the oplus operation wrt tau.
    )"
  );

  py_class.def(
    "lplus",
    &_LieGroup::template lplus<Tangent>,
    pybind11::arg("tau"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_mout_tau", OptJacobianRef(), "None"),
    R"(
      Left oplus operation of the Lie group.

      See Eq. (27).

      Parameters
      ----------
      tau : Lie group tangent
        An element of the tangent of the Lie group.
      J_out_self [out] : numpy.ndarray
        Jacobian of the oplus operation wrt self.
      J_out_tau [out] : numpy.ndarray
        Jacobian of the oplus operation wrt tau.
    )"
  );

  py_class.def(
    "plus",
    &_LieGroup::template plus<Tangent>,
    pybind11::arg("tau"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_mout_tau", OptJacobianRef(), "None"),
    "An alias for the 'rplus' function."
  );

  py_class.def(
    "rminus",
    &_LieGroup::template rminus<_LieGroup>,
    pybind11::arg("other"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_out_other", OptJacobianRef(), "None"),
    R"(
      Right ominus operation of the Lie group.

      See Eq. (26).

      Parameters
      ----------
      other : Lie group
        Another element of the same Lie group.
      J_out_self [out] : numpy.ndarray
        Jacobian of the ominus operation wrt self.
      J_out_other [out] : numpy.ndarray
        Jacobian of the ominus operation wrt other.
    )"
  );

  py_class.def(
    "lminus",
    &_LieGroup::template lminus<_LieGroup>,
    pybind11::arg("other"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_out_other", OptJacobianRef(), "None"),
    R"(
      Left ominus operation of the Lie group.

      See Eq. (28).

      Parameters
      ----------
      other : Lie group
        Another element of the same Lie group.
      J_out_self [out] : numpy.ndarray
        Jacobian of the ominus operation wrt self.
      J_out_other [out] : numpy.ndarray
        Jacobian of the ominus operation wrt other.
    )"
  );

  py_class.def(
    "minus",
    &_LieGroup::template minus<_LieGroup>,
    pybind11::arg("other"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_out_other", OptJacobianRef(), "None"),
    "An alias for the 'rminus' function."
  );

  py_class.def(
    "setIdentity",
    &_LieGroup::setIdentity,
    "Set self to the Lie group Identity."
  );

  py_class.def(
    "setRandom",
    &_LieGroup::setRandom,
    "Set self to a random value."
  );

  py_class.def_static(
    "Identity",
    &_LieGroup::Identity,
    "Static helper to create an object set at the Lie group Identity."
  );

  py_class.def_static(
    "Random",
    &_LieGroup::Random,
    "Static helper to create a random object of the Lie group."
  );

  py_class.def(
    pybind11::self + Tangent(),
    "Operator overload for the 'plus' function."
    )
    // .def(pybind11::self += Tangent())
    .def(
      pybind11::self - pybind11::self,
      "Operator overload for the 'minus' function."
    )
    .def(
      pybind11::self * pybind11::self,
      "Operator overload for the 'compose' function."
    )
    // .def(pybind11::self *= pybind11::self)
    .def(
      pybind11::self == pybind11::self,
      "Operator overload for the 'isApprox' function."
    )
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
