#ifndef _MANIF_MANIF_SO2_H_
#define _MANIF_MANIF_SO2_H_

#include "manif/impl/so2/SO2_base.h"

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct SO2;
template <typename _Scalar> struct SO2Tangent;

namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<SO2<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = SO2<_Scalar>;
  using Tangent  = SO2Tangent<_Scalar>;

  using Base = SO2Base<SO2<_Scalar>>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = 2;

  using DataType       = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, 3, 3>;
  using Rotation       = Eigen::Matrix<Scalar, Dim, Dim>;
  using Vector         = Eigen::Matrix<Scalar, Dim, 1>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif {

//
// LieGroup
//

/**
 * @brief Represents an element of SO2.
 */
template <typename _Scalar>
struct SO2 : SO2Base<SO2<_Scalar>>
{
private:

  using Base = SO2Base<SO2<_Scalar>>;
  using Type = SO2<_Scalar>;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::normalize;

  SO2()  = default;
  ~SO2() = default;

  // Copy constructor given base
  SO2(const Base& o);

  template <typename _DerivedOther>
  SO2(const SO2Base<_DerivedOther>& o);

  template <typename _DerivedOther>
  SO2(const LieGroupBase<_DerivedOther>& o);

  // Copy constructor given Eigen
  template <typename _EigenDerived>
  SO2(const Eigen::MatrixBase<_EigenDerived>& data);

  /**
   * @brief Constructor given the real and imaginary part
   * of a unit complex number representing the angle.
   * @param[in] real The real of a unitary complex number.
   * @param[in] imag The imaginary of a unitary complex number.
   * @throws manif::invalid_argument on un-normalized complex number.
   */
  SO2(const Scalar real, const Scalar imag);

  //! @brief Constructor given an angle (rad.)
  SO2(const Scalar theta);

  // LieGroup common API

  //! Get a const reference to the underlying DataType.
  const DataType& coeffs() const;

  // SO2 specific API

  using Base::angle;

protected:

  friend struct LieGroupBase<SO2<Scalar>>;
  DataType& coeffs_nonconst();

  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(SO2)

template <typename _Scalar>
template <typename _EigenDerived>
SO2<_Scalar>::SO2(const Eigen::MatrixBase<_EigenDerived>& data)
  : data_(data)
{
  using std::abs;
  MANIF_CHECK(abs(data_.norm()-Scalar(1)) < Constants<Scalar>::eps_s,
              "SO2 constructor argument not normalized !",
              invalid_argument);
}

template <typename _Scalar>
SO2<_Scalar>::SO2(const Base& o)
  : SO2(o.coeffs())
{
  //
}

template <typename _Scalar>
template <typename _DerivedOther>
SO2<_Scalar>::SO2(
    const SO2Base<_DerivedOther>& o)
  : SO2(o.coeffs())
{
  //
}

template <typename _Scalar>
template <typename _DerivedOther>
SO2<_Scalar>::SO2(
    const LieGroupBase<_DerivedOther>& o)
  : SO2(o.coeffs())
{
  //
}

template <typename _Scalar>
SO2<_Scalar>::SO2(const Scalar real, const Scalar imag)
  : SO2(DataType(real, imag))
{
  //
}

template <typename _Scalar>
SO2<_Scalar>::SO2(const Scalar theta)
  : SO2(cos(theta), sin(theta))
{
  using std::cos;
  using std::sin;
}

template <typename _Scalar>
typename SO2<_Scalar>::DataType&
SO2<_Scalar>::coeffs_nonconst()
{
  return data_;
}

template <typename _Scalar>
const typename SO2<_Scalar>::DataType&
SO2<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_H_ */
