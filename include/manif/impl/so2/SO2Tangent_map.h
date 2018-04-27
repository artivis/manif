#ifndef _MANIF_MANIF_SO2TANGENT_MAP_H_
#define _MANIF_MANIF_SO2TANGENT_MAP_H_

#include "manif/impl/so2/SO2Tangent.h"

namespace manif {
namespace internal {

template <>
template <typename _Scalar>
struct traits< Eigen::Map<SO2Tangent<_Scalar>,0> >
    : public traits<SO2Tangent<_Scalar>>
{
  using typename traits<SO2Tangent<_Scalar>>::Scalar;
  using traits<SO2Tangent<_Scalar>>::RepSize;
  using ManifoldDataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

template <>
template <typename _Scalar>
struct traits< Eigen::Map<const SO2Tangent<_Scalar>,0> >
    : public traits<const SO2Tangent<_Scalar>>
{
  using typename traits<const SO2Tangent<_Scalar>>::Scalar;
  using traits<const SO2Tangent<_Scalar>>::RepSize;
  using ManifoldDataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::SO2Tangent<_Scalar>, 0>
    : public manif::SO2TangentBase<Map<manif::SO2Tangent<_Scalar>, 0> >
{
  using Base = manif::SO2TangentBase<Map<manif::SO2Tangent<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  ManifoldDataType const* data() const { return &data_; }

protected:

  friend class manif::ManifoldBase<Map<manif::SO2Tangent<_Scalar>, 0>>;
  ManifoldDataType* data() { return &data_; }

  ManifoldDataType data_;
};

template <class _Scalar>
class Map<const manif::SO2Tangent<_Scalar>, 0>
    : public manif::SO2TangentBase<Map<const manif::SO2Tangent<_Scalar>, 0> >
{
  using Base = manif::SO2TangentBase<Map<const manif::SO2Tangent<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

//  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  ManifoldDataType const* data() const { return &data_; }

protected:

  friend class manif::ManifoldBase<Map<const manif::SO2Tangent<_Scalar>, 0>>;
  ManifoldDataType* data() { return &data_; }

  ManifoldDataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SO2TANGENT_MAP_H_ */
