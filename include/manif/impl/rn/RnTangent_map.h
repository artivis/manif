#ifndef _MANIF_MANIF_RNTANGENT_MAP_H_
#define _MANIF_MANIF_RNTANGENT_MAP_H_

#include "manif/impl/rn/RnTangent.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar, unsigned int _N>
struct traits< Eigen::Map<RnTangent<_Scalar, _N>,0> >
    : public traits<RnTangent<_Scalar, _N>>
{
  using typename traits<RnTangent<_Scalar, _N>>::Scalar;
  using traits<RnTangent<_Scalar, _N>>::DoF;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = RnTangentBase<Eigen::Map<RnTangent<Scalar, _N>, 0>>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar, unsigned int _N>
struct traits< Eigen::Map<const RnTangent<_Scalar, _N>,0> >
    : public traits<const RnTangent<_Scalar, _N>>
{
  using typename traits<const RnTangent<_Scalar, _N>>::Scalar;
  using traits<const RnTangent<_Scalar, _N>>::DoF;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = RnTangentBase<const Eigen::Map<RnTangent<Scalar, _N>, 0>>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

//! @brief Specialization of Map for manif::RnTangent
template <class _Scalar, unsigned int _N>
class Map<manif::RnTangent<_Scalar, _N>, 0>
    : public manif::RnTangentBase<Map<manif::RnTangent<_Scalar, _N>, 0> >
{
  using Base = manif::RnTangentBase<Map<manif::RnTangent<_Scalar, _N>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_TANGENT_MAP_ASSIGN_OP(RnTangent)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

//! @brief Specialization of Map for const manif::RnTangent
template <class _Scalar, unsigned int _N>
class Map<const manif::RnTangent<_Scalar, _N>, 0>
    : public manif::RnTangentBase<Map<const manif::RnTangent<_Scalar, _N>, 0> >
{
  using Base = manif::RnTangentBase<Map<const manif::RnTangent<_Scalar, _N>, 0> >;

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

#endif // _MANIF_MANIF_RNTANGENT_MAP_H_
