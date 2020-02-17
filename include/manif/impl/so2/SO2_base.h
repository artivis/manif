#ifndef _MANIF_MANIF_SO2_BASE_H_
#define _MANIF_MANIF_SO2_BASE_H_

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the SO2 group.
 * @note See Appendix A of the paper.
 */
template <typename _Derived>
struct SO2Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = SO2Base<_Derived>;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SO2Base)

public:

  MANIF_GROUP_TYPEDEF
  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Transformation = typename internal::traits<_Derived>::Transformation;

  MANIF_GROUP_API

  using Base::coeffs;
  using Base::data;

  MANIF_GROUP_ML_ASSIGN_OP(SO2Base)
  MANIF_GROUP_OPERATOR

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref log instead.
   */
  MANIF_DEPRECATED
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief Get the transformation matrix (2D isometry).
   * @note T = | R 0 |
   *           | 0 1 |
   */
  Transformation transform() const;

  /**
   * @brief Get the rotation matrix R.
   */
  Rotation rotation() const;

  /**
   * @brief Get the real part of the underlying complex number.
   */
  Scalar real() const;

  /**
   * @brief Get the imaginary part of the underlying complex number.
   */
  Scalar imag() const;

  /**
   * @brief Get the angle (rad.).
   */
  Scalar angle() const;

  /**
   * @brief Normalize the underlying complex number.
   */
  void normalize();

// protected:

  /// @todo given a Eigen::Map<const SO2>
  /// coeffs()->x() return a reference to
  /// temporary ...

//  Scalar& real();
//  Scalar& imag();
};

template <typename _Derived>
typename SO2Base<_Derived>::Transformation
SO2Base<_Derived>::transform() const
{
  Transformation T(Transformation::Identity());
  T.template block<2,2>(0,0) = rotation();
  return T;
}

template <typename _Derived>
typename SO2Base<_Derived>::Rotation
SO2Base<_Derived>::rotation() const
{
  using std::sin;
  using std::cos;
  const Scalar theta = angle();
  return (Rotation() << cos(theta), -sin(theta),
                        sin(theta),  cos(theta)).finished();
}

template <typename _Derived>
typename SO2Base<_Derived>::Tangent
SO2Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

template <typename _Derived>
/*const*/ typename SO2Base<_Derived>::Scalar/*&*/
SO2Base<_Derived>::real() const
{
  return coeffs().x();
}

template <typename _Derived>
/*const*/ typename SO2Base<_Derived>::Scalar/*&*/
SO2Base<_Derived>::imag() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SO2Base<_Derived>::Scalar
SO2Base<_Derived>::angle() const
{
  using std::atan2;
  return atan2(imag(), real());
}

//template <typename _Derived>
//typename SO2Base<_Derived>::Scalar&
//SO2Base<_Derived>::real()
//{
//  return coeffs.x();
//}

//template <typename _Derived>
//typename SO2Base<_Derived>::Scalar&
//SO2Base<_Derived>::imag()
//{
//  return coeffs.y();
//}

template <typename _Derived>
void SO2Base<_Derived>::normalize()
{
  coeffs().normalize();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_BASE_H_ */
