#ifndef _MANIF_MANIF_DHU_MAP_H_
#define _MANIF_MANIF_DHU_MAP_H_

#include "manif/impl/dhu/DHu.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<DHu<_Scalar>,0> >
    : public traits<DHu<_Scalar>>
{
  using typename traits<DHu<_Scalar>>::Scalar;
  using traits<DHu<Scalar>>::RepSize;
  using Base = DHuBase<Eigen::Map<DHu<Scalar>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const DHu<_Scalar>,0> >
    : public traits<const DHu<_Scalar>>
{
  using typename traits<const DHu<_Scalar>>::Scalar;
  using traits<const DHu<Scalar>>::RepSize;
  using Base = DHuBase<Eigen::Map<const DHu<Scalar>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

/**
 * @brief Specialization of Map for manif::DHu
 */
template <class _Scalar>
class Map<manif::DHu<_Scalar>, 0>
    : public manif::DHuBase<Map<manif::DHu<_Scalar>, 0> >
{
  using Base = manif::DHuBase<Map<manif::DHu<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_GROUP_MAP_ASSIGN_OP(DHu)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::DHu
 */
template <class _Scalar>
class Map<const manif::DHu<_Scalar>, 0>
    : public manif::DHuBase<Map<const manif::DHu<_Scalar>, 0> >
{
  using Base = manif::DHuBase<Map<const manif::DHu<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} // namespace Eigen

#endif // _MANIF_MANIF_DHU_MAP_H_
