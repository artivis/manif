#ifndef _MANIF_MANIF_SE2_MAP_H_
#define _MANIF_MANIF_SE2_MAP_H_

#include "manif/impl/se2/SE2.h"

namespace manif {
namespace internal {

template <typename _Scalar>
struct traits< Eigen::Map<SE2<_Scalar>,0> >
    : public traits<SE2<_Scalar>>
{
  using typename traits<SE2<_Scalar>>::Scalar;
  using traits<SE2<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

template <typename _Scalar>
struct traits< Eigen::Map<const SE2<_Scalar>,0> >
    : public traits<const SE2<_Scalar>>
{
  using typename traits<const SE2<_Scalar>>::Scalar;
  using traits<const SE2<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen {

template <class _Scalar>
class Map<manif::SE2<_Scalar>, 0>
    : public manif::SE2Base<Map<manif::SE2<_Scalar>, 0> >
{
  using Base = manif::SE2Base<Map<manif::SE2<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

  using Base::angle;
  using Base::real;
  using Base::imag;
  using Base::x;
  using Base::y;

protected:

  friend class manif::LieGroupBase<Map<manif::SE2<_Scalar>, 0>>;
  DataType& coeffs_nonconst() { return data_; }

  DataType data_;
};

template <class _Scalar>
class Map<const manif::SE2<_Scalar>, 0>
    : public manif::SE2Base<Map<const manif::SE2<_Scalar>, 0> >
{
  using Base = manif::SE2Base<Map<const manif::SE2<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

  using Base::angle;
  using Base::real;
  using Base::imag;
  using Base::x;
  using Base::y;

protected:

  const DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SE2_MAP_H_ */
