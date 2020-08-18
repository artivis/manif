#ifndef _MANIF_MANIF_DHU_BASE_H_
#define _MANIF_MANIF_DHU_BASE_H_

#include "manif/impl/dhu/DHu_properties.h"
#include "manif/impl/lie_group_base.h"

namespace manif {

//
// LieGroup
//

/**
 * @brief The base class of the DHu group (Unit Dual Quaternion).
 * @note s = [r,d] = [r0,r1,r2,r3, d0,d1,d2,d3]
 * with r (real) is a quaternion q = [x,y,z,w] and
 * with d (dual) is a quaternion q = [x,y,z,w].
 * A dual quaternion is unit if s . s* = 1, that is if
 * |r| = 1
 * and
 * r dot d = 0
 */
template <typename _Derived>
struct DHuBase : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = DHuBase<_Derived>;

public:

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Translation    = typename internal::traits<_Derived>::Translation;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using Isometry       = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
  using Quaternion     = Eigen::Quaternion<Scalar>;
  using Real           = Quaternion;
  using Dual           = Quaternion;

  // LieGroup common API

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(DHuBase)

public:

  MANIF_GROUP_ML_ASSIGN_OP(DHuBase)

  /**
   * @brief Get the inverse.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the DHu corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The DHu tangent of this.
   * @note This is the log() map in vector form.
   * @see DHuTangent.
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
   * @brief Composition of this and another DHu element.
   * @param[in] m Another DHu element.
   * @param[out] -optional- J_mc_ma Jacobian of the composition wrt this.
   * @param[out] -optional- J_mc_mb Jacobian of the composition wrt m.
   * @return The composition of 'this . m'.
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
   */
  template <typename _EigenDerived>
  Eigen::Matrix<Scalar, 3, 1>
  act(const Eigen::MatrixBase<_EigenDerived> &v,
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 6>>> J_vout_m = {},
      tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_v = {}) const;

  /**
   * @brief Get the adjoint matrix of DHu at this.
   */
  Jacobian adj() const;

  // DHu specific functions

  LieGroup conjugate() const;
  LieGroup conjugatedual() const;

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
   * @brief Get the real part of this as a quaternion.
   */
  Real real() const;

  /**
   * @brief Get the dual part of this as a quaternion.
   */
  Dual dual() const;

  /**
   * @brief Get the translational part in vector form.
   */
  Translation translation() const;

  /**
   * @brief Normalize the underlying dual quaternion.
   */
  void normalize();
};

template <typename _Derived>
typename DHuBase<_Derived>::LieGroup
DHuBase<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  if (J_minv_m)
  {
    (*J_minv_m) = -adj();
  }

  return conjugate();
}

template <typename _Derived>
typename DHuBase<_Derived>::Tangent
DHuBase<_Derived>::log(OptJacobianRef J_t_m) const
{
  // @todo
  Tangent tan;

  if (J_t_m)
  {
    // Jr^-1
    (*J_t_m) = tan.rjacinv();
  }

  return tan;
}

template <typename _Derived>
typename DHuBase<_Derived>::Tangent
DHuBase<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename DHuBase<_Derived>::LieGroup
DHuBase<_Derived>::compose(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  static_assert(
    std::is_base_of<DHuBase<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from DHuBase !");
  
  const auto& m_DHu = static_cast<const DHuBase<_DerivedOther>&>(m);

  if (J_mc_ma)
  {
    (*J_mc_ma) = m_DHu.inverse().adj();
  }

  if (J_mc_mb)
  {
    J_mc_mb->setIdentity();
  }

  return LieGroup(real()*m_DHu.real(),
                  Quaternion((real()*m_DHu.dual()).coeffs()+(dual()*m_DHu.real()).coeffs()));
}

template <typename _Derived>
template <typename _EigenDerived>
Eigen::Matrix<typename DHuBase<_Derived>::Scalar, 3, 1>
DHuBase<_Derived>::act(const Eigen::MatrixBase<_EigenDerived> &v,
                       tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 6>>> J_vout_m,
                       tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>> J_vout_v) const
{
  assert_vector_dim(v, 3);
  const Rotation R(rotation());

  if (J_vout_m)
  {
    // @todo
  }

  if (J_vout_v)
  {
    // @todo
  }

  // @todo do it in a dual quat fashion
  return translation() + R * v;
}


template <typename _Derived>
typename DHuBase<_Derived>::Jacobian
DHuBase<_Derived>::adj() const
{
  // @todo

  return Jacobian();
}

// DHu specific function

template <typename _Derived>
typename DHuBase<_Derived>::Transformation
DHuBase<_Derived>::transform() const
{
  Transformation T = Transformation::Identity();
  T.template topLeftCorner<3,3>()  = rotation();
  T.template topRightCorner<3,1>() = translation();
  return T;
}

template <typename _Derived>
typename DHuBase<_Derived>::Isometry
DHuBase<_Derived>::isometry() const
{
  return Isometry(transform());
}

template <typename _Derived>
typename DHuBase<_Derived>::Rotation
DHuBase<_Derived>::rotation() const
{
  return real().matrix();
}

template <typename _Derived>
typename DHuBase<_Derived>::Real
DHuBase<_Derived>::real() const
{
  return Real(coeffs().template head<4>());
}

template <typename _Derived>
typename DHuBase<_Derived>::Dual
DHuBase<_Derived>::dual() const
{
  return Real(coeffs().template tail<4>());
}

template <typename _Derived>
typename DHuBase<_Derived>::Translation
DHuBase<_Derived>::translation() const
{
  return dual()*real().conjugate()*Scalar(2);
}

template <typename _Derived>
typename DHuBase<_Derived>::LieGroup
DHuBase<_Derived>::conjugate() const
{
  return LieGroup(real().conjugate(), dual().conjugate());
}

template <typename _Derived>
typename DHuBase<_Derived>::LieGroup
DHuBase<_Derived>::conjugatedual() const
{
  return LieGroup((DataType() << coeffs().template head<4>(), 
                                -coeffs().template tail<4>()).finished());
}

template <typename _Derived>
void DHuBase<_Derived>::normalize()
{
  const Scalar n = coeffs().template head<4>().norm();
  coeffs() /= n;
  // real and dual parts are orthogonal
  coeffs().template tail<4>() -= (coeffs().template head<4>().dot(coeffs().template tail<4>())) * coeffs().template head<4>();
}

namespace internal {

//! @brief Random specialization for DHuBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<DHuBase<Derived>>
{
  template <typename T>
  static void run(T& m)
  {
    // @note:
    // Quaternion::UnitRandom is not available in Eigen 3.3-beta1
    // which is the default version in Ubuntu 16.04
    // So we copy its implementation here.

    using std::sqrt;
    using std::sin;
    using std::cos;

    using Scalar      = typename DHuBase<Derived>::Scalar;
    using LieGroup    = typename DHuBase<Derived>::LieGroup;
    using DataType    = typename DHuBase<Derived>::DataType;

    Scalar u1 = Eigen::internal::random<Scalar>(0, 1),
           u2 = Eigen::internal::random<Scalar>(0, 2*EIGEN_PI),
           u3 = Eigen::internal::random<Scalar>(0, 2*EIGEN_PI);
    Scalar a = sqrt(Scalar(1) - u1),
           b = sqrt(u1);

    Eigen::Matrix<Scalar, 4, 1> real(a * cos(u2),b * sin(u3),b * cos(u3),a * sin(u2));

    u1 = Eigen::internal::random<Scalar>(0, 1);
    u2 = Eigen::internal::random<Scalar>(0, 2*EIGEN_PI);
    u3 = Eigen::internal::random<Scalar>(0, 2*EIGEN_PI);
    a = sqrt(Scalar(1) - u1);
    b = sqrt(u1);

    Eigen::Matrix<Scalar, 4, 1> dual(a * cos(u2),b * sin(u3),b * cos(u3),a * sin(u2));

    dual -= real.dot(dual) * real;

    m = LieGroup((DataType() << real, dual).finished());
    
    // @note see https://scicomp.stackexchange.com/a/27843

    // Scalar x = a * cos(u2),
    //        y = b * sin(u3),
    //        z = b * cos(u3),
    //        w = a * sin(u2);

    // Eigen::Matrix<Scalar, 4, 3> M;
    // M << -x, -y, -z,
    //       w,  z, -y,
    //      -z,  w,  x,
    //       y, -x,  w;

    // const Eigen::Matrix<Scalar, 4, 1> v = M * Eigen::Matrix<Scalar, 3, 1>::Random().normalized();

    // m = LieGroup((DataType() << x, y, z, w, v(1), v(2), v(3), v(0)).finished());
  }
};

//! @brief Assignment assert specialization for SE2Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<DHuBase<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)
  {
    using std::abs;
    using Scalar = typename DHuBase<Derived>::Scalar;
    // |r| = 1
    MANIF_CHECK(abs(data.template head<4>().norm()-Scalar(1)) <
                Constants<Scalar>::eps_s,
                "DHu real part not normalized !",
                invalid_argument);
    // r dot d = 0
    MANIF_CHECK(abs(data.template head<4>().dot(data.template tail<4>())) <
                Constants<Scalar>::eps_s,
                "DHu dual part not normalized !",
                invalid_argument);
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_DHU_BASE_H_
