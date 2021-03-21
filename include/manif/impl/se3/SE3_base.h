#ifndef _MANIF_MANIF_SE3_BASE_H_
#define _MANIF_MANIF_SE3_BASE_H_

#include "manif/impl/se3/SE3_properties.h"
#include "manif/impl/lie_group_base.h"
#include "manif/impl/so3/SO3_map.h"

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

public:

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Translation    = typename internal::traits<_Derived>::Translation;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using Isometry       = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

  using QuaternionDataType = Eigen::Quaternion<Scalar>;

  // LieGroup common API

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SE3Base)

public:

  MANIF_GROUP_ML_ASSIGN_OP(SE3Base)

  /**
   * @brief Get the inverse.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   * @note See Eqs. (170,176).
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the SE3 corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The SE3 tangent of this.
   * @note This is the log() map in vector form.
   * @note See Eq. (173) & Eq. (79,179,180) and following notes.
   * @see SE3Tangent.
   */
  Tangent log(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref log instead.
   */
  MANIF_DEPRECATED
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief Composition of this and another SE3 element.
   * @param[in] m Another SE3 element.
   * @param[out] -optional- J_mc_ma Jacobian of the composition wrt this.
   * @param[out] -optional- J_mc_mb Jacobian of the composition wrt m.
   * @return The composition of 'this . m'.
   * @note See Eq. (171) and Eqs. (177,178).
   */
  template <typename _DerivedOther>
  LieGroup compose(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  /**
   * @brief Rigid motion action on a 3D point.
   * @param  v A 3D point.
   * @param[out] -optional- J_vout_m The Jacobian of the new object wrt this.
   * @param[out] -optional- J_vout_v The Jacobian of the new object wrt input object.
   * @return The transformed 3D point.
   * @note See Eq. (181) & Eqs. (182,183).
   */
  template <typename _EigenDerived>
  Eigen::Matrix<Scalar, 3, 1>
  act(const Eigen::MatrixBase<_EigenDerived> &v,
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 6>>> J_vout_m = {},
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_v = {}) const;

  /**
   * @brief Get the adjoint matrix of SE3 at this.
   * @note See Eq. (175).
   */
  Jacobian adj() const;

  // SE3 specific functions

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

  /**
   * @brief Get the rotational part of this as a rotation matrix.
   */
  Rotation rotation() const;

  /**
   * @brief Get the rotational part of this as a quaternion.
   */
  QuaternionDataType quat() const;

  /**
   * @brief Get the translational part in vector form.
   */
  Translation translation() const;

  /**
   * @brief Get the x component of the translational part.
   */
  Scalar x() const;

  /**
   * @brief Get the y component of translational part.
   */
  Scalar y() const;

  /**
   * @brief Get the z component of translational part.
   */
  Scalar z() const;

  //Scalar roll() const;
  //Scalar pitch() const;
  //Scalar yaw() const;

  /**
   * @brief Normalize the underlying quaternion.
   */
  void normalize();

  /**
   * @brief Set the rotational as a quaternion.
   * @param quaternion a unitary quaternion
   */
  void quat(const QuaternionDataType& quaternion);

  /**
   * @brief Set the rotational as a quaternion.
   * @param quaternion an Eigen::Vector representing a unitary quaternion
   */
  template <typename _EigenDerived>
  void quat(const Eigen::MatrixBase<_EigenDerived>& quaternion);

  /**
   * @brief Set the rotational as a so3 object.
   * @param so3 a manif::SO3 object
   */
  void quat(const SO3<Scalar>& so3);

  /**
   * @brief Set the translation of the SE3 object
   * @param translation, 3d-vector representing the translation
   */
  void translation(const Translation& translation);

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
void SE3Base<_Derived>::quat(const QuaternionDataType& quaternion)
{
  quat(quaternion.coeffs());
}

template <typename _Derived>
template <typename _EigenDerived>
void SE3Base<_Derived>::quat(const Eigen::MatrixBase<_EigenDerived>& quaternion)
{
  using std::abs;
  assert_vector_dim(quaternion, 4);
  MANIF_ASSERT(abs(quaternion.norm()-Scalar(1)) <
               Constants<Scalar>::eps_s,
               "The quaternion is not normalized !",
               invalid_argument);

  asSO3().coeffs() = quaternion;
}

template <typename _Derived>
void SE3Base<_Derived>::quat(const SO3<Scalar>& so3)
{
  quat(so3.coeffs());
}

template <typename _Derived>
void SE3Base<_Derived>::translation(const Translation& translation)
{
  coeffs().template head<3>() = translation;
}

template <typename _Derived>
typename SE3Base<_Derived>::LieGroup
SE3Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  if (J_minv_m)
  {
    (*J_minv_m) = -adj();
  }

  const SO3<Scalar> so3inv = asSO3().inverse();

  return LieGroup(-so3inv.act(translation()),
                   so3inv);
}

template <typename _Derived>
typename SE3Base<_Derived>::Tangent
SE3Base<_Derived>::log(OptJacobianRef J_t_m) const
{
  using std::abs;
  using std::sqrt;

  const SO3Tangent<Scalar> so3tan = asSO3().log();

  Tangent tan((typename Tangent::DataType() <<
               so3tan.ljacinv()*translation(),
               so3tan.coeffs()).finished());

  if (J_t_m)
  {
    // Jr^-1
    (*J_t_m) = tan.rjacinv();
  }

  return tan;
}

template <typename _Derived>
typename SE3Base<_Derived>::Tangent
SE3Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename SE3Base<_Derived>::LieGroup
SE3Base<_Derived>::compose(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  static_assert(
    std::is_base_of<SE3Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from SE3Base !");

  const auto& m_se3 = static_cast<const SE3Base<_DerivedOther>&>(m);

  if (J_mc_ma)
  {
    (*J_mc_ma) = m.inverse().adj();
  }

  if (J_mc_mb)
  {
    J_mc_mb->setIdentity();
  }

  return LieGroup(rotation()*m_se3.translation() + translation(),
                  asSO3().compose(m_se3.asSO3()).quat());
}

template <typename _Derived>
template <typename _EigenDerived>
Eigen::Matrix<typename SE3Base<_Derived>::Scalar, 3, 1>
SE3Base<_Derived>::act(const Eigen::MatrixBase<_EigenDerived> &v,
                       tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 6>>> J_vout_m,
                       tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_v) const
{
  assert_vector_dim(v, 3);
  const Rotation R(rotation());

  if (J_vout_m)
  {
    J_vout_m->template topLeftCorner<3,3>()  =  R;
    J_vout_m->template topRightCorner<3,3>() = -R * skew(v);
  }

  if (J_vout_v)
  {
    (*J_vout_v) = R;
  }

  return translation() + R * v;
}


template <typename _Derived>
typename SE3Base<_Derived>::Jacobian
SE3Base<_Derived>::adj() const
{
  /// @note Chirikjian (close to Eq.10.94)
  /// says
  ///       Ad(g) = |  R  0 |
  ///               | T.R R |
  ///
  /// considering vee(log(g)) = (w;v)
  /// with T = [t]_x
  ///
  /// but this is
  ///       Ad(g) = | R T.R |
  ///               | 0  R  |
  ///
  /// considering vee(log(g)) = (v;w)

  Jacobian Adj = Jacobian::Zero();
  Adj.template topLeftCorner<3,3>() = rotation();
  Adj.template bottomRightCorner<3,3>() =
      Adj.template topLeftCorner<3,3>();
  Adj.template topRightCorner<3,3>() =
    skew(translation()) * Adj.template topLeftCorner<3,3>();

  return Adj;
}

// SE3 specific function

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

namespace internal {

//! @brief Random specialization for SE3Base objects.
template <typename Derived>
struct RandomEvaluatorImpl<SE3Base<Derived>>
{
  template <typename T>
  static void run(T& m)
  {
    using Scalar      = typename SE3Base<Derived>::Scalar;
    using Translation = typename SE3Base<Derived>::Translation;
    using LieGroup    = typename SE3Base<Derived>::LieGroup;

    m = LieGroup(Translation::Random(), randQuat<Scalar>());
  }
};

//! @brief Assignment assert specialization for SE2Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<SE3Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)
  {
    using std::abs;
    MANIF_ASSERT(
      abs(data.template tail<4>().norm()-typename SE3Base<Derived>::Scalar(1)) <
      Constants<typename SE3Base<Derived>::Scalar>::eps_s,
      "SE3 assigned data not normalized !",
      manif::invalid_argument
    );
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_SE3_BASE_H_ */
