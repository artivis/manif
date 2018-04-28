#ifndef _MANIF_MANIF_SO2_MAP_H_
#define _MANIF_MANIF_SO2_MAP_H_

#include "manif/impl/so2/SO2.h"

namespace manif {
namespace internal {

template <>
template <typename _Scalar>
struct traits< Eigen::Map<SO2<_Scalar>,0> >
    : public traits<SO2<_Scalar>>
{
  using typename traits<SO2<_Scalar>>::Scalar;
  using traits<SO2<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

template <>
template <typename _Scalar>
struct traits< Eigen::Map<const SO2<_Scalar>,0> >
    : public traits<const SO2<_Scalar>>
{
  using typename traits<const SO2<_Scalar>>::Scalar;
  using traits<const SO2<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
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
  using Type = Map<manif::SO2<_Scalar>, 0>;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  DataType const* data() const { return &data_; }

protected:

  friend class manif::ManifoldBase<Map<manif::SO2<_Scalar>, 0>>;
  DataType* data() { return &data_; }

  DataType data_;
};

template <class _Scalar>
class Map<const manif::SO2<_Scalar>, 0>
    : public manif::SO2Base<Map<const manif::SO2<_Scalar>, 0> >
{
  using Base = manif::SO2Base<Map<const manif::SO2<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  DataType const* data() const { return &data_; }

protected:

  friend class manif::ManifoldBase<Map<const manif::SO2<_Scalar>, 0>>;
  DataType* data() { return &data_; }

  DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SO2_MAP_H_ */
