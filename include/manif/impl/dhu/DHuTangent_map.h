#ifndef _MANIF_MANIF_DHUTANGENT_MAP_H_
#define _MANIF_MANIF_DHUTANGENT_MAP_H_

#include "manif/impl/dhu/DHuTangent.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<DHuTangent<_Scalar>,0> >
    : public traits<DHuTangent<_Scalar>>
{
  using typename traits<DHuTangent<_Scalar>>::Scalar;
  using traits<DHuTangent<_Scalar>>::DoF;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = DHuTangentBase<Eigen::Map<DHuTangent<Scalar>, 0>>;
};

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<const DHuTangent<_Scalar>,0> >
    : public traits<const DHuTangent<_Scalar>>
{
  using typename traits<const DHuTangent<_Scalar>>::Scalar;
  using traits<const DHuTangent<_Scalar>>::DoF;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = DHuTangentBase<Eigen::Map<const DHuTangent<Scalar>, 0>>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

/**
 * @brief Specialization of Map for manif::DHu
 */
template <class _Scalar>
class Map<manif::DHuTangent<_Scalar>, 0>
    : public manif::DHuTangentBase<Map<manif::DHuTangent<_Scalar>, 0> >
{
  using Base = manif::DHuTangentBase<Map<manif::DHuTangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_TANGENT_MAP_ASSIGN_OP(DHuTangent)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::DHu
 */
template <class _Scalar>
class Map<const manif::DHuTangent<_Scalar>, 0>
    : public manif::DHuTangentBase<Map<const manif::DHuTangent<_Scalar>, 0> >
{
  using Base = manif::DHuTangentBase<Map<const manif::DHuTangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} // namespace Eigen

#endif // _MANIF_MANIF_DHUTANGENT_MAP_H_
