#ifndef _MANIF_MANIF_SE2_H_
#define _MANIF_MANIF_SE2_H_

#include "manif/impl/se2/SE2_base.h"

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct SE2;
template <typename _Scalar> struct SE2Tangent;

namespace internal {

//! traits specialization
template <typename _Scalar>
struct traits<SE2<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = SE2<_Scalar>;
  using Tangent  = SE2Tangent<_Scalar>;

  using Base = SE2Base<SE2<_Scalar>>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = 4;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Basis          = Eigen::Matrix<Scalar, 3, 3>;
  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, 3, 3>;
  using Rotation       = Eigen::Matrix<Scalar, Dim, Dim>;
  using Translation    = Eigen::Matrix<Scalar, Dim, 1>;
  using Vector         = Eigen::Matrix<Scalar, DoF, 1>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif {

////////////////
///          ///
/// LieGroup ///
///          ///
////////////////

/**
 * @brief Represents an element of SE2.
 */
template <typename _Scalar>
struct SE2 : SE2Base<SE2<_Scalar>>
{
private:

  using Base = SE2Base<SE2<_Scalar>>;
  using Type = SE2<_Scalar>;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API

  SE2()  = default;
  ~SE2() = default;

  // Copy constructor given base
  SE2(const Base& o);

  template <typename _DerivedOther>
  SE2(const SE2Base<_DerivedOther>& o);

  template <typename _DerivedOther>
  SE2(const LieGroupBase<_DerivedOther>& o);

  // Copy constructor given Eigen
  template <typename _EigenDerived>
  SE2(const Eigen::MatrixBase<_EigenDerived>& data);

  SE2(const Scalar x, const Scalar y, const Scalar theta);
  SE2(const Scalar x, const Scalar y, const Scalar real, const Scalar imag);

  /// LieGroup common API

  const DataType& coeffs() const;

  /// SE2 specific API

  using Base::angle;
  using Base::real;
  using Base::imag;
  using Base::x;
  using Base::y;

protected:

  friend class LieGroupBase<SE2<Scalar>>;
  DataType& coeffs_nonconst();

  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(SE2)

template <typename _Scalar>
SE2<_Scalar>::SE2(const Base& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar>
template <typename _DerivedOther>
SE2<_Scalar>::SE2(
    const SE2Base<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar>
template <typename _DerivedOther>
SE2<_Scalar>::SE2(
    const LieGroupBase<_DerivedOther>& o)
  : data_(o.coeffs())
{
  //
}

template <typename _Scalar>
template <typename _EigenDerived>
SE2<_Scalar>::SE2(const Eigen::MatrixBase<_EigenDerived>& data)
  : data_(data)
{
  //
}

template <typename _Scalar>
SE2<_Scalar>::SE2(const Scalar x, const Scalar y, const Scalar theta)
  : SE2(DataType(x, y, cos(theta), sin(theta)))
{
  using std::cos;
  using std::sin;
}

template <typename _Scalar>
SE2<_Scalar>::SE2(const Scalar x, const Scalar y,
                  const Scalar real, const Scalar imag)
  : SE2(DataType(x, y, real, imag))
{
  //
}

template <typename _Scalar>
typename SE2<_Scalar>::DataType&
SE2<_Scalar>::coeffs_nonconst()
{
  return data_;
}

template <typename _Scalar>
const typename SE2<_Scalar>::DataType&
SE2<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE2_H_ */
