#ifndef _MANIF_MANIF_SO2TANGENT_H_
#define _MANIF_MANIF_SO2TANGENT_H_

#include "manif/impl/so2/SO2Tangent_base.h"

#include <Eigen/Core>

namespace manif
{
namespace internal
{

// Traits specialization

template <typename _Scalar>
struct traits<SO2Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SO2<_Scalar>;
  using Tangent  = SO2Tangent<_Scalar>;

  using Base = SO2TangentBase<_Scalar>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType  = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Scalar>
struct SO2Tangent : SO2TangentBase<SO2Tangent<_Scalar>>
{
  using Base = SO2TangentBase<SO2Tangent<_Scalar>>;

  using Scalar   = typename Base::Scalar;
  using Manifold = typename Base::Manifold;
  using DataType = typename Base::DataType;

  SO2Tangent() = default;

  SO2Tangent(const double theta);
  SO2Tangent(const DataType& theta);

  /// Tangent common API

  DataType* data();
  const DataType* data() const;

  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  /// SO2Tangent specific API

  using Base::angle;

  /// @todo consider using a
  /// Eigen::Matrix<std::complex<Scalar>, 1, 1>
  /// as DataType
//  Scalar angle2()
//  {
//    using std::atan2;
//    Eigen::Matrix<std::complex<Scalar>, 1, 1> test;
//    return atan2(test.imag()(0), test.real()(0));
//  }

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SO2Tangent);

template <typename _Scalar>
SO2Tangent<_Scalar>::SO2Tangent(const double theta)
  : data_(theta)
{
  //
}

template <typename _Scalar>
SO2Tangent<_Scalar>::SO2Tangent(const DataType& theta)
  : data_(theta)
{
  //
}

template <typename _Scalar>
typename SO2Tangent<_Scalar>::DataType*
SO2Tangent<_Scalar>::data()
{
  return &data_;
}

template <typename _Scalar>
const typename SO2Tangent<_Scalar>::DataType*
SO2Tangent<_Scalar>::data() const
{
  return &data_;
}

}

/// Eigen Map

namespace manif {
namespace internal {

//template <>
//template <typename _Scalar>
//struct traits< Eigen::Map<SO2<_Scalar>,0> >
//    : public traits<SO2<_Scalar>>
//{
//  using typename traits<SO2<_Scalar>>::Scalar;
//  using traits<SO2<_Scalar>>::RepSize;
//  using ManifoldDataType = ::Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>,0>;
//};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

/// @todo

/*
template <class _Scalar>
class Map<manif::SO2<_Scalar>, 0>
    : public manif::SO2Base<Map<manif::SO2<_Scalar>, 0> >
{
  using Base = manif::SO2Base<Map<manif::SO2<_Scalar>, 0> >;

public:

  using Tangent = typename Base::Tangent;

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

//  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)

  //  using Base::data;
  using Base::matrix;
  using Base::rotation;
  using Base::identity;
  using Base::random;
  using Base::inverse;
  using Base::rplus;
  using Base::lplus;
  using Base::rminus;
  using Base::lminus;
  using Base::lift;
  using Base::compose;

  Map(Scalar* coeffs) : data_(coeffs) { }

  template <typename _Derived>
  Manifold& operator =(const manif::SO2Base<_Derived>& o)
  {
    data_ = *o.data();
    return *this;
  }

  ManifoldDataType const* data() const { return &data_; }

protected:

  friend class manif::ManifoldBase<Map<manif::SO2<_Scalar>, 0>>;

  ManifoldDataType* data() { return &data_; }
  ManifoldDataType data_;
};

template <class _Scalar>
class Map<const manif::SO2<_Scalar>, 0>
    : public manif::SO2Base<Map<const manif::SO2<_Scalar>, 0> >
{
  using Base = manif::SO2Base<Map<const manif::SO2<_Scalar>, 0> >;

public:

  using Tangent = typename Base::Tangent;

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

//  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)

  //  using Base::data;
  using Base::matrix;
  using Base::rotation;
  using Base::identity;
  using Base::random;
  using Base::inverse;
  using Base::rplus;
  using Base::lplus;
  using Base::rminus;
  using Base::lminus;
  using Base::lift;
  using Base::compose;

  Map(Scalar* coeffs) : data_(coeffs) { }

  ManifoldDataType const* data() const { return &data_; }

protected:

  friend class manif::ManifoldBase<Map<manif::SO2<_Scalar>, 0>>;

  ManifoldDataType* data() { return &data_; }
  ManifoldDataType data_;
};
*/
} /* namespace Eigen */

#endif /* _MANIF_MANIF_SO2TANGENT_H_ */
