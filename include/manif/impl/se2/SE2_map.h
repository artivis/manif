#ifndef _MANIF_MANIF_SE2_MAP_H_
#define _MANIF_MANIF_SE2_MAP_H_

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<SE2<_Scalar>,0> >
    : public traits<SE2<_Scalar>>
{
  using typename traits<SE2<_Scalar>>::Scalar;
  using traits<SE2<Scalar>>::RepSize;
  using Base = SE2Base<Eigen::Map<SE2<Scalar>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const SE2<_Scalar>,0> >
    : public traits<const SE2<_Scalar>>
{
  using typename traits<const SE2<_Scalar>>::Scalar;
  using traits<const SE2<Scalar>>::RepSize;
  using Base = SE2Base<Eigen::Map<const SE2<Scalar>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen {

/**
 * @brief Specialization of Map for manif::SE2
 */
template <class _Scalar>
class Map<manif::SE2<_Scalar>, 0>
    : public manif::SE2Base<Map<manif::SE2<_Scalar>, 0> >
{
  using Base = manif::SE2Base<Map<manif::SE2<_Scalar>, 0> >;

public:

  MANIF_GROUP_TYPEDEF

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_GROUP_API

  MANIF_GROUP_MAP_ASSIGN_OP(SE2)
  MANIF_GROUP_OPERATOR

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }
  using Base::data;

  // SE2 specific API

  using Base::transform;
  using Base::isometry;
  using Base::rotation;
  using Base::translation;
  using Base::normalize;
  using Base::angle;
  using Base::real;
  using Base::imag;
  using Base::x;
  using Base::y;

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::SE2
 */
template <class _Scalar>
class Map<const manif::SE2<_Scalar>, 0>
    : public manif::SE2Base<Map<const manif::SE2<_Scalar>, 0> >
{
  using Base = manif::SE2Base<Map<const manif::SE2<_Scalar>, 0> >;

public:

  MANIF_GROUP_TYPEDEF

  Map(const Scalar* coeffs) : data_(coeffs) { }

  MANIF_GROUP_CONST_API
  const DataType& coeffs() const { return data_; }
  using Base::data;

  MANIF_GROUP_CONST_OPERATOR

  using Base::transform;
  using Base::isometry;
  using Base::rotation;
  using Base::translation;
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
