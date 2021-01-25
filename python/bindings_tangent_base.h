#ifndef _MANIF_PYTHON_BINDINGS_TANGENT_BASE_H_
#define _MANIF_PYTHON_BINDINGS_TANGENT_BASE_H_

template <typename _Tangent, typename... _Args>
void wrap_tangent_base(pybind11::class_<_Tangent, _Args...>& py_class) {

  using Scalar = typename _Tangent::Scalar;
  using LieGroup = typename _Tangent::LieGroup;
  using DataType = typename _Tangent::DataType;
  using Jacobian = typename _Tangent::Jacobian;
  using OptJacobianRef = typename _Tangent::OptJacobianRef;

  py_class.attr("Dim") = _Tangent::Dim;
  py_class.attr("DoF") = _Tangent::DoF;
  py_class.attr("RepSize") = _Tangent::RepSize;

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
    static_cast<DataType& (_Tangent::*)(void)>(&_Tangent::coeffs),
    "Return a copy of underlying data."
  ); // Makes a copy!

  py_class.def(
    "coeffs",
    static_cast<DataType& (_Tangent::*)(void)>(&_Tangent::coeffs),
    pybind11::return_value_policy::reference_internal,
    "Get a reference to underlying data."
  );

  py_class.def(
    "generator",
    &_Tangent::generator,
    pybind11::arg("i"),
    "Get the ith basis element of the Lie Algebra."
  );

  py_class.def(
    "innerWeights",
    &_Tangent::innerWeights,
    "Get the weight matrix of the Weighted Euclidean inner product, "
    "relative to the space basis."
  );

  py_class.def(
    "inner",
    &_Tangent::template inner<_Tangent>,
    pybind11::arg("other"),
    R"(
      Get inner product of this and another Tangent weighted by W.

      ret = self^T x W x other
    )"
  );

  py_class.def(
    "weightedNorm",
    &_Tangent::weightedNorm,
    "Get the Euclidean weighted norm."
  );

  py_class.def(
    "squaredWeightedNorm",
    &_Tangent::squaredWeightedNorm,
    "Get the squared Euclidean weighted norm."
  );

  py_class.def(
    "hat",
    &_Tangent::hat,
    R"(
      Get the isomorphic element in the Lie algebra.

      See Eq. (10).
    )"
  );

  py_class.def(
    "exp",
    &_Tangent::exp,
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    R"(
      Get the corresponding Lie group element.

      Eq. (23).

      Parameters
      ----------
      J_out_self [out] : numpy.ndarray
        Jacobian of the log wrt self.
    )"
  );

  py_class.def(
    "rplus",
    &_Tangent::rplus,
    pybind11::arg("state"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_out_state", OptJacobianRef(), "None"),
    R"(
      Right oplus operation of the Lie group.

      See Eqs. (25).

      Parameters
      ----------
      other : Lie group
        Another object of the same Lie group.
      J_out_self [out] : numpy.ndarray
        Jacobian of the oplus operation wrt self.
      J_out_state [out] : numpy.ndarray
        Jacobian of the oplus operation wrt state.
    )"
  );

  py_class.def(
    "lplus",
    &_Tangent::lplus,
    pybind11::arg("state"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_out_state", OptJacobianRef(), "None"),
    R"(
      Left oplus operation of the Lie group.

      See Eqs. (27).

      Parameters
      ----------
      other : Lie group
        Another object of the same Lie group.
      J_out_self [out] : numpy.ndarray
        Jacobian of the oplus operation wrt self.
      J_out_state [out] : numpy.ndarray
        Jacobian of the oplus operation wrt state.
    )"
  );

  py_class.def(
    "plus",
    static_cast<LieGroup (_Tangent::*)(const LieGroup&, OptJacobianRef, OptJacobianRef) const>(&_Tangent::plus),
    pybind11::arg("state"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_out_state", OptJacobianRef(), "None"),
    "An alias for the 'rplus' function."
  );

  py_class.def(
    "plus",
    &_Tangent::template plus<_Tangent>,
    pybind11::arg("other"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_out_other", OptJacobianRef(), "None"),
    R"(
      Plus operation in the vector space.

      Parameters
      ----------
      other : Lie group tangent
        Another object of the same Lie group tangent.
      J_out_self [out] : numpy.ndarray
        Jacobian of the oplus operation wrt self.
      J_out_other [out] : numpy.ndarray
        Jacobian of the oplus operation wrt other.
    )"
  );

  py_class.def(
    "minus",
    &_Tangent::template minus<_Tangent>,
    pybind11::arg("other"),
    pybind11::arg_v("J_out_self", OptJacobianRef(), "None"),
    pybind11::arg_v("J_out_other", OptJacobianRef(), "None"),
    R"(
      Minus operation in the vector space.

      Parameters
      ----------
      other : Lie group tangent
        Another object of the same Lie group tangent.
      J_out_self [out] : numpy.ndarray
        Jacobian of the oplus operation wrt self.
      J_out_other [out] : numpy.ndarray
        Jacobian of the oplus operation wrt other.
    )"
  );

  py_class.def(
    "rjac",
    &_Tangent::rjac,
    R"(
      Get the right Jacobian.

      This is the right Jacobian of 'exp',
      what is commonly known as "the right Jacobian".

      See Eq. (41) for the right Jacobian of general functions.
      See Eqs. (126,143,163,179,191) for implementations of the
      right Jacobian of exp.
    )"
  );

  py_class.def(
    "ljac",
    &_Tangent::ljac,
    R"(
      Get the left Jacobian.

      This is the left Jacobian of 'exp',
      what is commonly known as "the left Jacobian".

      See Eq. (44) for the left Jacobian of general functions.
      See Eqs. (126,145,164,179,191) for implementations of the
      left Jacobian of exp.
    )"
  );

  // py_class.def("rjacinv", &_Tangent::rjacinv);
  // py_class.def("ljacinv", &_Tangent::ljacinv);

  py_class.def("smallAdj", &_Tangent::smallAdj);

  py_class.def(
    "isApprox",
    [](const _Tangent& self, const _Tangent& t, Scalar eps) {
      return self.isApprox(t, eps);
    },
    pybind11::arg("other"),
    pybind11::arg_v("eps", Scalar(manif::Constants<Scalar>::eps), "1e-10"),
    R"(
      Evaluate whether self and other are 'close'.

      Parameters
      ----------
      other : Lie group tangent
        Another object of the same Lie group tangent.
      eps : double
        Threshold for equality comparison. Default: 1e-10.
    )"
  );

  py_class.def(
    "isApprox",
    [](const _Tangent& self, const DataType& t, Scalar eps) {
      return self.isApprox(t, eps);
    },
    pybind11::arg("other"),
    pybind11::arg_v("eps", Scalar(manif::Constants<Scalar>::eps), "1e-10"),
    R"(
      Evaluate whether self and other are 'close'.

      Parameters
      ----------
      other : numpy.array
        Another object of the same Lie group tangent.
      eps : double
        Threshold for equality comparison. Default: 1e-10.
    )"
  );

  py_class.def(
    "setZero",
    &_Tangent::setZero,
    "Set self to zero."
  );

  py_class.def(
    "setRandom",
    &_Tangent::setRandom,
    "Set self to a random value."
  );

  py_class.def_static(
    "Zero",
    &_Tangent::Zero,
    "Static helper to create an object of the Lie group tangent set to zero."
  );

  py_class.def_static(
    "Random",
    &_Tangent::Random,
    "Static helper to create a random object of the Lie group."
  );

  py_class.def_static(
    "Generator",
    &_Tangent::Generator,
    pybind11::arg("i"),
    "Static helper to get the ith basis element of the Lie Algebra."
  );

  py_class.def_static(
    "InnerWeights",
    &_Tangent::InnerWeights,
    "Static helper to get the weight matrix of the "
    "Weighted Euclidean inner product, "
    "relative to the space basis."
  );

  // operator overloads
  py_class.def(-pybind11::self)
    .def(
      pybind11::self + LieGroup(),
      "Operator overload for the 'plus' function."
    )
    .def(
      pybind11::self + pybind11::self,
      "Operator overload for the 'plus' function."
    )
    // .def(pybind11::self += pybind11::self)
    .def(
      pybind11::self - pybind11::self,
      "Operator overload for the 'minus' function."
    )
    // .def(pybind11::self -= pybind11::self)
    .def(
      pybind11::self * Scalar(),
      "Multiply the vector by a scalar."
    )
    .def(
      Scalar() * pybind11::self,
      "Multiply the vector by a scalar."
    )
    .def(
      pybind11::self / Scalar(),
      "Divide the vector by a scalar."
    )
    .def(
      pybind11::self == pybind11::self,
      "Operator overload for the 'isApprox' function."
    )
    ;

  // Jacobian() @ pybind11::self
  py_class.def(
    "__rmatmul__",
    [](const _Tangent& t, pybind11::array_t<Scalar> lhs) {

    pybind11::buffer_info lhs_buf = lhs.request();

    if (lhs_buf.ndim != 2)
        throw std::runtime_error("Number of dimensions must be 2");

    if (lhs_buf.size != _Tangent::DoF * _Tangent::DoF)
        throw std::runtime_error("Input shapes must match");

    _Tangent result = Eigen::Map<Jacobian>(static_cast<Scalar*>(lhs_buf.ptr)) * t;

    return result;

    },
    pybind11::is_operator()
  );

  // Jacobian() * pybind11::self
  py_class.def(
    "__rmul__",
    [](const _Tangent& t, pybind11::array_t<Scalar> lhs) {

    pybind11::buffer_info lhs_buf = lhs.request();

    // if (lhs_buf.ndim != 2)
        // throw std::runtime_error("Number of dimensions must be 2");

    if (lhs_buf.size != _Tangent::DoF * _Tangent::DoF)
        throw std::runtime_error("Input shapes must match");

    _Tangent result = Eigen::Map<Jacobian>(static_cast<Scalar*>(lhs_buf.ptr)) * t;

    return result;

    },
    pybind11::is_operator()
  );

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
