#ifndef _MANIF_MANIF_RN_H_
#define _MANIF_MANIF_RN_H_

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar, unsigned int N> struct Rn;
template <typename _Scalar, unsigned int N> struct RnTangent;

namespace internal {

//! Traits specialization
template <typename _Scalar, unsigned int _N>
struct traits<Rn<_Scalar, _N>>
{
  using Scalar = _Scalar;

  using LieGroup = Rn<_Scalar, _N>;
  using Tangent  = RnTangent<_Scalar, _N>;

  using Base = RnBase<Rn<_Scalar, _N>>;

  static constexpr int Dim     = _N;
  static constexpr int DoF     = _N;
  static constexpr int RepSize = _N;

  using DataType       = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, DoF, DoF>;
  using Vector         = Eigen::Matrix<Scalar, DoF, 1>;
};

} // namespace internal
} // namespace manif

namespace manif {

//
// LieGroup
//

/**
 * @brief Represents an element of Rn.
 */
template <typename _Scalar, unsigned int _N>
struct Rn : RnBase<Rn<_Scalar, _N>>
{
private:

  static_assert(_N > 0, "N must be greater than 0 !");

  using Base = RnBase<Rn<_Scalar, _N>>;
  using Type = Rn<_Scalar, _N>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_GROUP_TYPEDEF
  using typename Base::Transformation;

  Rn()  = default;
  ~Rn() = default;

  // Copy constructor given base
  MANIF_COPY_CONSTRUCTOR(Rn)
  template <typename _DerivedOther>
  Rn(const LieGroupBase<_DerivedOther>& o);

  MANIF_GROUP_API
  using Base::data;

  MANIF_COEFFS_FUNCTIONS

  MANIF_GROUP_ASSIGN_OP(Rn)
  MANIF_GROUP_OPERATOR

protected:

  DataType data_;
};

template <typename _Scalar> using R1 = Rn<_Scalar, 1>;
template <typename _Scalar> using R2 = Rn<_Scalar, 2>;
template <typename _Scalar> using R3 = Rn<_Scalar, 3>;
template <typename _Scalar> using R4 = Rn<_Scalar, 4>;
template <typename _Scalar> using R5 = Rn<_Scalar, 5>;
template <typename _Scalar> using R6 = Rn<_Scalar, 6>;
template <typename _Scalar> using R7 = Rn<_Scalar, 7>;
template <typename _Scalar> using R8 = Rn<_Scalar, 8>;
template <typename _Scalar> using R9 = Rn<_Scalar, 9>;

MANIF_EXTRA_GROUP_TYPEDEF(R1)
MANIF_EXTRA_GROUP_TYPEDEF(R2)
MANIF_EXTRA_GROUP_TYPEDEF(R3)
MANIF_EXTRA_GROUP_TYPEDEF(R4)
MANIF_EXTRA_GROUP_TYPEDEF(R5)
MANIF_EXTRA_GROUP_TYPEDEF(R6)
MANIF_EXTRA_GROUP_TYPEDEF(R7)
MANIF_EXTRA_GROUP_TYPEDEF(R8)
MANIF_EXTRA_GROUP_TYPEDEF(R9)

template <typename _Scalar, unsigned int _N>
template <typename _DerivedOther>
Rn<_Scalar, _N>::Rn(const LieGroupBase<_DerivedOther>& o)
  : Rn(o.coeffs())
{
  //
}

} /* namespace manif */

#endif /* _MANIF_MANIF_RN_H_ */
