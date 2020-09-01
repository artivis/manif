#ifndef _MANIF_MANIF_SO2_MAP_H_
#define _MANIF_MANIF_SO2_MAP_H_

#include "manif/impl/so2/SO2.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<SO2<_Scalar>,0> >
    : public traits<SO2<_Scalar>>
{
  using typename traits<SO2<_Scalar>>::Scalar;
  using traits<SO2<Scalar>>::RepSize;
  using Base = SO2Base<Eigen::Map<SO2<Scalar>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const SO2<_Scalar>,0> >
    : public traits<const SO2<_Scalar>>
{
  using typename traits<const SO2<_Scalar>>::Scalar;
  using traits<const SO2<Scalar>>::RepSize;
  using Base = SO2Base<Eigen::Map<const SO2<Scalar>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen {

/**
 * @brief Specialization of Map for manif::SO2
 */
template <class _Scalar>
class Map<manif::SO2<_Scalar>, 0>
    : public manif::SO2Base<Map<manif::SO2<_Scalar>, 0> >
{
  using Base = manif::SO2Base<Map<manif::SO2<_Scalar>, 0> >;

public:

  MANIF_GROUP_TYPEDEF

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_GROUP_API

  MANIF_GROUP_MAP_ASSIGN_OP(SO2)
  MANIF_GROUP_OPERATOR

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }
  using Base::data;

  using Base::transform;
  using Base::rotation;
  using Base::real;
  using Base::imag;
  using Base::angle;
  using Base::normalize;

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::SO2
 */
template <class _Scalar>
class Map<const manif::SO2<_Scalar>, 0>
    : public manif::SO2Base<Map<const manif::SO2<_Scalar>, 0> >
{
  using Base = manif::SO2Base<Map<const manif::SO2<_Scalar>, 0> >;

public:

  MANIF_GROUP_TYPEDEF

  Map(const Scalar* coeffs) : data_(coeffs) { }

  MANIF_GROUP_CONST_API
  const DataType& coeffs() const { return data_; }
  using Base::data;

  using Base::transform;
  using Base::rotation;
  using Base::real;
  using Base::imag;
  using Base::angle;

protected:

  const DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SO2_MAP_H_ */
