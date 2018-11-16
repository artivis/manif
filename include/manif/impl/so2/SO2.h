#ifndef _MANIF_MANIF_SO2_H_
#define _MANIF_MANIF_SO2_H_

#include "manif/impl/so2/SO2_base.h"

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct SO2;
template <typename _Scalar> struct SO2Tangent;

namespace internal {

// Traits specialization

template <typename _Scalar>
struct traits<SO2<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = SO2<_Scalar>;
  using Tangent  = SO2Tangent<_Scalar>;

  using Base = SO2Base<SO2<_Scalar>>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int N       = LieGroupProperties<Base>::N;
  static constexpr int RepSize = 2;

  /// @todo move those to some traits ?

  using DataType       = Eigen::Matrix<Scalar, RepSize, 1>;
  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, N, N>;
  using Rotation       = Eigen::Matrix<Scalar, Dim, Dim>;
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

template <typename _Scalar>
struct SO2 : SO2Base<SO2<_Scalar>>
{
private:

  using Base = SO2Base<SO2<_Scalar>>;
  using Type = SO2<_Scalar>;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF

  SO2()  = default;
  ~SO2() = default;

  SO2(const DataType& d);
  SO2(const Scalar real, const Scalar imag);
  SO2(const Scalar theta);

  /// LieGroup common API

  const DataType& coeffs() const;

  MANIF_INHERIT_GROUP_API

  /// SO2 specific API

  using Base::angle;

protected:

  friend class LieGroupBase<SO2<Scalar>>;
  DataType& coeffs_nonconst();

  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(SO2)

template <typename _Scalar>
SO2<_Scalar>::SO2(const DataType& d)
  : data_(d)
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
