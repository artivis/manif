#ifndef _MANIF_MANIF_SE3TANGENT_MAP_H_
#define _MANIF_MANIF_SE3TANGENT_MAP_H_

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<SE3Tangent<_Scalar>,0> >
    : public traits<SE3Tangent<_Scalar>>
{
  using typename traits<SE3Tangent<_Scalar>>::Scalar;
  using traits<SE3Tangent<_Scalar>>::DoF;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = SE3TangentBase<Eigen::Map<SE3Tangent<Scalar>, 0>>;
};

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<const SE3Tangent<_Scalar>,0> >
    : public traits<const SE3Tangent<_Scalar>>
{
  using typename traits<const SE3Tangent<_Scalar>>::Scalar;
  using traits<const SE3Tangent<_Scalar>>::DoF;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = SE3TangentBase<Eigen::Map<const SE3Tangent<Scalar>, 0>>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen {

/**
 * @brief Specialization of Map for manif::SE3
 */
template <class _Scalar>
class Map<manif::SE3Tangent<_Scalar>, 0>
    : public manif::SE3TangentBase<Map<manif::SE3Tangent<_Scalar>, 0> >
{
  using Base = manif::SE3TangentBase<Map<manif::SE3Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_TANGENT_API
  using Base::data;
  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

  MANIF_TANGENT_MAP_ASSIGN_OP(SE3Tangent)
  MANIF_TANGENT_OPERATOR

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::SE3
 */
template <class _Scalar>
class Map<const manif::SE3Tangent<_Scalar>, 0>
    : public manif::SE3TangentBase<Map<const manif::SE3Tangent<_Scalar>, 0> >
{
  using Base = manif::SE3TangentBase<Map<const manif::SE3Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF

  Map(const Scalar* coeffs) : data_(coeffs) { }

  MANIF_TANGENT_CONST_API
  const DataType& coeffs() const { return data_; }
  const Scalar* data() const { return data_; }

  MANIF_TANGENT_CONST_OPERATOR

protected:

  const DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SE3TANGENT_MAP_H_ */
