#ifndef _MANIF_MANIF_SO2_MAP_H_
#define _MANIF_MANIF_SO2_MAP_H_

#include "manif/impl/SO2.h"

namespace manif {
namespace internal {

template <>
template <typename _Scalar>
struct traits< Eigen::Map<SO2<_Scalar>,0> >
    : public traits<SO2<_Scalar>>
{
  using typename traits<SO2<_Scalar>>::Scalar;
  using traits<SO2<_Scalar>>::RepSize;
  using ManifoldDataType = ::Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

template <>
template <typename _Scalar>
struct traits< Eigen::Map<const SO2<_Scalar>,0> >
    : public traits<const SO2<_Scalar>>
{
  using typename traits<const SO2<_Scalar>>::Scalar;
  using traits<const SO2<_Scalar>>::RepSize;
  using ManifoldDataType = ::Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::SO2<_Scalar>, 0>
    : public manif::SO2Base<Map<manif::SO2<_Scalar>, 0> >
{
  using Base = manif::SO2Base<Map<manif::SO2<_Scalar>, 0> >;

public:

  using Tangent = typename Base::Tangent;

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

//  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)

  MANIF_INHERIT_MANIFOLD_API

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

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  ManifoldDataType const* data() const { return &data_; }

protected:

  friend class manif::ManifoldBase<Map<const manif::SO2<_Scalar>, 0>>;
  ManifoldDataType* data() { return &data_; }

  ManifoldDataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SO2_MAP_H_ */
