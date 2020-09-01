#ifndef _MANIF_MANIF_SE2_BASE_H_
#define _MANIF_MANIF_SE2_BASE_H_

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the SE2 group.
 * @note See Appendix C of the paper.
 */
template <typename _Derived>
struct SE2Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = SE2Base<_Derived>;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SE2Base)

public:

  MANIF_GROUP_TYPEDEF
  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Translation    = typename internal::traits<_Derived>::Translation;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using Isometry       = Eigen::Transform<Scalar, 2, Eigen::Isometry>;

  MANIF_GROUP_API

  using Base::coeffs;
  using Base::data;

  MANIF_GROUP_ML_ASSIGN_OP(SE2Base)
  MANIF_GROUP_OPERATOR

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref log instead.
   */
  MANIF_DEPRECATED
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  // SE2 specific functions

  /**
   * @brief Get the transformation matrix (2D isometry).
   * @note T = | R t |
   *           | 0 1 |
   */
  Transformation transform() const;

  /**
   * Get the isometry object (Eigen 2D isometry).
   * @note T = | R t |
   *           | 0 1 |
   */
  Isometry isometry() const;

  //! @brief Get the rotational part of this as a rotation matrix.
  Rotation rotation() const;

  //! @brief Get the translational part of this as a vector.
  Translation translation() const;

  /**
   * @brief Get the real part of the underlying complex number representing
   * the rotational part.
   */
  Scalar real() const;

  /**
   * @brief Get the imaginary part of the underlying complex number representing
   * the rotational part.
   */
  Scalar imag() const;

  /**
   * @brief Get the angle (rad.) of the rotational part.
   */
  Scalar angle() const;

  /**
   * @brief Get the x component of the translational part.
   */
  Scalar x() const;

  /**
   * @brief Get the y component of the translational part.
   */
  Scalar y() const;

  /**
   * @brief Normalize the underlying complex number.
   */
  void normalize();
};

template <typename _Derived>
typename SE2Base<_Derived>::Transformation
SE2Base<_Derived>::transform() const
{
  Transformation T(Transformation::Identity());
  T.template block<2,2>(0,0) = rotation();
  T(0,2) = x();
  T(1,2) = y();
  return T;
}

template <typename _Derived>
typename SE2Base<_Derived>::Isometry
SE2Base<_Derived>::isometry() const
{
  return Isometry(transform());
}

template <typename _Derived>
typename SE2Base<_Derived>::Rotation
SE2Base<_Derived>::rotation() const
{
  return (Rotation() << real(), -imag(),
                        imag(),  real() ).finished();
}

template <typename _Derived>
typename SE2Base<_Derived>::Translation
SE2Base<_Derived>::translation() const
{
  return Translation(x(), y());
}

template <typename _Derived>
typename SE2Base<_Derived>::Tangent
SE2Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

// SE2 specific function

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::real() const
{
  return coeffs()(2);
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::imag() const
{
  return coeffs()(3);
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::angle() const
{
  using std::atan2;
  return atan2(imag(), real());
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SE2Base<_Derived>::Scalar
SE2Base<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
void SE2Base<_Derived>::normalize()
{
  coeffs().template tail<2>().normalize();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE2_BASE_H_ */
