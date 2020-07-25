#ifndef _MANIF_MANIF_RN_MAP_H_
#define _MANIF_MANIF_RN_MAP_H_

#include "manif/impl/rn/Rn.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar, unsigned int _N>
struct traits< Eigen::Map<Rn<_Scalar, _N>,0> >
    : public traits<Rn<_Scalar, _N>>
{
  using typename traits<Rn<_Scalar, _N>>::Scalar;
  using traits<Rn<Scalar, _N>>::RepSize;
  using Base = RnBase<Eigen::Map<Rn<Scalar, _N>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar, unsigned int _N>
struct traits< Eigen::Map<const Rn<_Scalar, _N>,0> >
    : public traits<const Rn<_Scalar, _N>>
{
  using typename traits<const Rn<_Scalar, _N>>::Scalar;
  using traits<const Rn<Scalar, _N>>::RepSize;
  using Base = RnBase<Eigen::Map<const Rn<Scalar, _N>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

/**
 * @brief Specialization of Map for manif::Rn
 */
template <class _Scalar, unsigned int _N>
class Map<manif::Rn<_Scalar, _N>, 0>
    : public manif::RnBase<Map<manif::Rn<_Scalar, _N>, 0> >
{
  using Base = manif::RnBase<Map<manif::Rn<_Scalar, _N>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_GROUP_MAP_ASSIGN_OP(Rn)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::Rn
 */
template <class _Scalar, unsigned int _N>
class Map<const manif::Rn<_Scalar, _N>, 0>
    : public manif::RnBase<Map<const manif::Rn<_Scalar, _N>, 0> >
{
  using Base = manif::RnBase<Map<const manif::Rn<_Scalar, _N>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} // namespace Eigen

#endif // _MANIF_MANIF_RN_MAP_H_
