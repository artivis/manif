#ifndef _MANIF_MANIF_SE3_BASE_H_
#define _MANIF_MANIF_SE3_BASE_H_

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the SE3 group.
 * @note See Appendix D of the paper.
 */
template <typename _Derived>
struct SE3Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = SE3Base<_Derived>;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SE3Base)

public:

  MANIF_GROUP_TYPEDEF
  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Translation    = typename internal::traits<_Derived>::Translation;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using Isometry       = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
  using QuaternionDataType = Eigen::Quaternion<Scalar>;

  MANIF_GROUP_API
  using Base::coeffs;
  using Base::data;

  MANIF_GROUP_ML_ASSIGN_OP(SE3Base)
  MANIF_GROUP_OPERATOR

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref log instead.
   */
  MANIF_DEPRECATED
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  /**
   * Get the transformation matrix (3D isometry).
   * @note T = | R t |
   *           | 0 1 |
   */
  Transformation transform() const;

  /**
   * Get the isometry object (Eigen 3D isometry).
   * @note T = | R t |
   *           | 0 1 |
   */
  Isometry isometry() const;

  //! @brief Get the rotational part of this as a rotation matrix.
  Rotation rotation() const;

  //! @brief Get the rotational part of this as a quaternion.
  QuaternionDataType quat() const;

  //! @brief Get the translational part in vector form.
  Translation translation() const;

  //! @brief Get the x component of the translational part.
  Scalar x() const;

  //! @brief Get the y component of translational part.
  Scalar y() const;

  //! @brief Get the z component of translational part.
  Scalar z() const;

  //! @brief Normalize the underlying quaternion.
  void normalize();

public: /// @todo make protected

  Eigen::Map<const SO3<Scalar>> asSO3() const
  {
    return Eigen::Map<const SO3<Scalar>>(coeffs().data()+3);
  }

  Eigen::Map<SO3<Scalar>> asSO3()
  {
    return Eigen::Map<SO3<Scalar>>(coeffs().data()+3);
  }
};

template <typename _Derived>
typename SE3Base<_Derived>::Transformation
SE3Base<_Derived>::transform() const
{
  Transformation T = Transformation::Identity();
  T.template topLeftCorner<3,3>()  = rotation();
  T.template topRightCorner<3,1>() = translation();
  return T;
}

template <typename _Derived>
typename SE3Base<_Derived>::Isometry
SE3Base<_Derived>::isometry() const
{
  return Isometry(transform());
}

template <typename _Derived>
typename SE3Base<_Derived>::Rotation
SE3Base<_Derived>::rotation() const
{
  return asSO3().rotation();
}

template <typename _Derived>
typename SE3Base<_Derived>::QuaternionDataType
SE3Base<_Derived>::quat() const
{
  return asSO3().quat();
}

template <typename _Derived>
typename SE3Base<_Derived>::Translation
SE3Base<_Derived>::translation() const
{
  return coeffs().template head<3>();
}

template <typename _Derived>
typename SE3Base<_Derived>::Tangent
SE3Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

template <typename _Derived>
typename SE3Base<_Derived>::Scalar
SE3Base<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SE3Base<_Derived>::Scalar
SE3Base<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SE3Base<_Derived>::Scalar
SE3Base<_Derived>::z() const
{
  return coeffs().z();
}

template <typename _Derived>
void SE3Base<_Derived>::normalize()
{
  coeffs().template tail<4>().normalize();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE3_BASE_H_ */
