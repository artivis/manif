#ifndef _MANIF_MANIF_SE3TANGENT_BASE_H_
#define _MANIF_MANIF_SE3TANGENT_BASE_H_

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the SE3 tangent.
 * @note See Appendix D of the paper.
 */
template <typename _Derived>
struct SE3TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SE3TangentBase<_Derived>;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(SE3TangentBase)

public:

  MANIF_TANGENT_TYPEDEF
  using BlockV = typename DataType::template FixedSegmentReturnType<3>::Type;
  using BlockW = typename DataType::template FixedSegmentReturnType<3>::Type;
  using ConstBlockV = typename DataType::template ConstFixedSegmentReturnType<3>::Type;
  using ConstBlockW = typename DataType::template ConstFixedSegmentReturnType<3>::Type;

  MANIF_TANGENT_API

  using Base::data;
  using Base::coeffs;

  MANIF_TANGENT_ML_ASSIGN_OP(SE3TangentBase)
  MANIF_TANGENT_OPERATOR

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref exp instead.
   */
  MANIF_DEPRECATED
  LieGroup retract(OptJacobianRef J_m_t = {}) const;

  //! @brief Get the linear part.
  BlockV v();
  const ConstBlockV v() const;

  //! @brief Get the angular part.
  BlockW w();
  const ConstBlockW w() const;

//  Scalar x() const;
//  Scalar y() const;
//  Scalar z() const;

  //Scalar roll() const;
  //Scalar pitch() const;
  //Scalar yaw() const;

public: /// @todo make protected

  const Eigen::Map<const SO3Tangent<Scalar>> asSO3() const
  {
    return Eigen::Map<const SO3Tangent<Scalar>>(coeffs().data()+3);
  }

  Eigen::Map<SO3Tangent<Scalar>> asSO3()
  {
    return Eigen::Map<SO3Tangent<Scalar>>(coeffs.data()+3);
  }

private:

  template <typename _EigenDerived>
  static void fillQ(Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>> Q,
                    const Eigen::MatrixBase<_EigenDerived>& c);
};

template <typename _Derived>
typename SE3TangentBase<_Derived>::LieGroup
SE3TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::BlockV
SE3TangentBase<_Derived>::v()
{
  return coeffs().template head<3>();
}

template <typename _Derived>
const typename SE3TangentBase<_Derived>::ConstBlockV
SE3TangentBase<_Derived>::v() const
{
  return coeffs().template head<3>();
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::BlockW SE3TangentBase<_Derived>::w()
{
  return coeffs().template tail<3>();
}

template <typename _Derived>
const typename SE3TangentBase<_Derived>::ConstBlockW
SE3TangentBase<_Derived>::w() const
{
  return coeffs().template tail<3>();
}

//template <typename _Derived>
//typename SE3TangentBase<_Derived>::Scalar
//SE3TangentBase<_Derived>::x() const
//{
//  return data()->x();
//}

//template <typename _Derived>
//typename SE3TangentBase<_Derived>::Scalar
//SE3TangentBase<_Derived>::y() const
//{
//  return data()->y();
//}

//template <typename _Derived>
//typename SE3TangentBase<_Derived>::Scalar
//SE3TangentBase<_Derived>::z() const
//{
//  return data()->z();
//}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE3_BASE_H_ */
