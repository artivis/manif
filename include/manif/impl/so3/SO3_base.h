#ifndef _MANIF_MANIF_SO3_BASE_H_
#define _MANIF_MANIF_SO3_BASE_H_

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the SO3 group.
 * @note See Appendix B of the paper.
 */
template <typename _Derived>
struct SO3Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = SO3Base<_Derived>;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SO3Base)

public:

  MANIF_GROUP_TYPEDEF
  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using QuaternionDataType = Eigen::Quaternion<Scalar>;

  MANIF_GROUP_API

  using Base::coeffs;
  using Base::data;

  MANIF_GROUP_ML_ASSIGN_OP(SO3Base)
  MANIF_GROUP_OPERATOR

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref log instead.
   */
  MANIF_DEPRECATED
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief Get the transformation matrix (3D isometry).
   * @note T = | R 0 |
   *           | 0 1 |
   */
  Transformation transform() const;

  //! @brief Get a rotation matrix.
  Rotation rotation() const;

  //! @brief Get the x component of the quaternion.
  Scalar x() const;
  //! @brief Get the y component of the quaternion.
  Scalar y() const;
  //! @brief Get the z component of the quaternion.
  Scalar z() const;
  //! @brief Get the w component of the quaternion.
  Scalar w() const;

  //! @brief Get quaternion.
  QuaternionDataType quat() const;

  //! @brief Normalize the underlying quaternion.
  void normalize();
};

template <typename _Derived>
typename SO3Base<_Derived>::Transformation
SO3Base<_Derived>::transform() const
{
  Transformation T = Transformation::Identity();
  T.template block<3,3>(0,0) = rotation();
  return T;
}

template <typename _Derived>
typename SO3Base<_Derived>::Rotation
SO3Base<_Derived>::rotation() const
{
  return quat().matrix();
}

template <typename _Derived>
typename SO3Base<_Derived>::Tangent
SO3Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::z() const
{
  return coeffs().z();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::w() const
{
  return coeffs().w();
}

template <typename _Derived>
typename SO3Base<_Derived>::QuaternionDataType
SO3Base<_Derived>::quat() const
{
  return QuaternionDataType(coeffs());
}

template <typename _Derived>
void SO3Base<_Derived>::normalize()
{
  coeffs().normalize();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3_BASE_H_ */
