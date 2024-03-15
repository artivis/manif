#ifndef _MANIF_MANIF_SGAL3TANGENT_MAP_H_
#define _MANIF_MANIF_SGAL3TANGENT_MAP_H_

#include "manif/impl/sgal3/SGal3Tangent.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits<Eigen::Map<SGal3Tangent<_Scalar>, 0> >
  : public traits<SGal3Tangent<_Scalar>> {
  using typename traits<SGal3Tangent<_Scalar>>::Scalar;
  using traits<SGal3Tangent<_Scalar>>::DoF;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = SGal3TangentBase<Eigen::Map<SGal3Tangent<Scalar>, 0>>;
};

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits<Eigen::Map<const SGal3Tangent<_Scalar>, 0> >
  : public traits<const SGal3Tangent<_Scalar>> {
  using typename traits<const SGal3Tangent<_Scalar>>::Scalar;
  using traits<const SGal3Tangent<_Scalar>>::DoF;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = SGal3TangentBase<Eigen::Map<const SGal3Tangent<Scalar>, 0>>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

/**
 * @brief Specialization of Map for manif::SGal3
 */
template <class _Scalar>
class Map<manif::SGal3Tangent<_Scalar>, 0>
  : public manif::SGal3TangentBase<Map<manif::SGal3Tangent<_Scalar>, 0> > {
  using Base = manif::SGal3TangentBase<Map<manif::SGal3Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_TANGENT_MAP_ASSIGN_OP(SGal3Tangent)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::SGal3
 */
template <class _Scalar>
class Map<const manif::SGal3Tangent<_Scalar>, 0>
  : public manif::SGal3TangentBase<Map<const manif::SGal3Tangent<_Scalar>, 0> > {
  using Base = manif::SGal3TangentBase<Map<const manif::SGal3Tangent<_Scalar>, 0> >;

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

#endif // _MANIF_MANIF_SGAL3TANGENT_MAP_H_
