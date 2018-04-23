#ifndef _MANIF_MANIF_SO3_H_
#define _MANIF_MANIF_SO3_H_

#include "manif/SO3_base.h"

#include <Eigen/Core>

#include <iostream>

namespace manif
{

// Forward declare for type traits specialization

template <typename _Scalar> struct SO3;
template <typename _Scalar> struct SO3Tangent;

namespace internal
{

// Traits specialization

template <>
template <typename _Scalar>
struct traits<SO3<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SO3<_Scalar>;
  using Tangent  = SO3<_Scalar>;

  using Base = SO3Base<SO3<_Scalar>>;

  static constexpr int Dim     = Base::Dim;
  static constexpr int RepSize = Base::RepSize;
  static constexpr int DoF     = Base::DoF;

  using ManifoldDataType = Eigen::Matrix<Scalar, 2, 1>;
  using TangentDataType  = Eigen::Matrix<Scalar, 2, 1>;

  using JacobianType = Eigen::Matrix<Scalar, 2, 2>;
  /*
  using JacobianType0    = Eigen::Matrix<Scalar, RepSize, RepSize>;
  using JacobianType1    = Eigen::Matrix<Scalar, RepSize, DoF>;
  using JacobianType2    = Eigen::Matrix<Scalar, DoF, RepSize>;
  */
};

template <typename _Scalar>
struct traits<SO3Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SO3<_Scalar>;
  using Tangent = SO3Tangent<_Scalar>;

  using Base = SO3Base<SO3<_Scalar>>;

  static constexpr int Dim     = Base::Dim;
  static constexpr int RepSize = Base::RepSize;
  static constexpr int DoF     = Base::DoF;

  using ManifoldDataType = Eigen::Matrix<Scalar, 2, 1>;
  using TangentDataType  = Eigen::Matrix<Scalar, 2, 1>;

  using JacobianType = Eigen::Matrix<Scalar, 2, 2>;
  /*
  using JacobianType0    = Eigen::Matrix<Scalar, RepSize, RepSize>;
  using JacobianType1    = Eigen::Matrix<Scalar, RepSize, DoF>;
  using JacobianType2    = Eigen::Matrix<Scalar, DoF, RepSize>;
  */
};

} /* namespace internal */

} /* namespace manif */

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Scalar>
struct SO3Tangent : SO3TangentBase<SO3Tangent<_Scalar>>
{
  SO3Tangent() = default;

  template <typename _OtherDerived>
  SO3Tangent(const ManifoldBase<_OtherDerived>& o);

//  SO3Tangent& operator=(const SO3& o);
//  SO3Tangent& operator=(SO3&& o);

  template <typename _OtherDerived>
  SO3Tangent& operator=(const ManifoldBase<_OtherDerived>& o);

  void zero();
  void random();
  SO3<_Scalar> retract() const;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SO3Tangent);

template <typename _Scalar>
template <typename _OtherDerived>
SO3Tangent<_Scalar>::SO3Tangent(const ManifoldBase<_OtherDerived>& o)
{

}

template <typename _Scalar>
template <typename _OtherDerived>
SO3Tangent<_Scalar>& SO3Tangent<_Scalar>::operator=(const ManifoldBase<_OtherDerived>& o)
{

}

template <typename _Scalar>
void SO3Tangent<_Scalar>::zero()
{
  std::cout << "SO3Tangent zero\n";
}

template <typename _Scalar>
void SO3Tangent<_Scalar>::random()
{
  std::cout << "SO3Tangent random\n";
}

template <typename _Scalar>
SO3<_Scalar>
SO3Tangent<_Scalar>::retract() const
{
  std::cout << "SO3Tangent rectract\n";
  return SO3<_Scalar>();
}

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Scalar>
struct SO3 : SO3Base<SO3<_Scalar>>
{
  using Base = SO3Base<SO3<_Scalar>>;

  using Tangent = typename Base::Tangent;

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  using ManifoldDataType2 = Eigen::Matrix<_Scalar, RepSize, 1>;
//  using TangentDataType  = Eigen::Matrix<_Scalar, 2, 1>;
//  using JacobianType = Eigen::Matrix<_Scalar, 2, 2>;

  SO3()  = default;
  ~SO3() = default;

//  SO3(const SO3& o);
//  SO3(SO3&& o);

  template <typename _OtherDerived>
  SO3(const ManifoldBase<_OtherDerived>& o);

//  SO3& operator=(const SO3& o);
//  SO3& operator=(SO3&& o);

  template <typename _OtherDerived>
  SO3& operator=(const ManifoldBase<_OtherDerived>& o);

  SO3(const ManifoldDataType& d);

  ManifoldDataType* data();

  const ManifoldDataType* data() const;

  void identity();

  void random();

  SO3 inverse() const;

  SO3 rplus(const SO3::Tangent& /*t*/) const;

  SO3 lplus(const SO3::Tangent& /*t*/) const;

  SO3 rminus(const SO3& /*m*/) const;

  SO3 lminus(const SO3& /*m*/) const;

  Tangent lift() const;

  SO3 compose(const SO3& /*m*/) const;

  ///

  void inverse(SO3& m, JacobianType& j) const;

protected:

  ManifoldDataType data_;
};

MANIF_EXTRA_MANIFOLD_TYPEDEF(SO3)

/// SO3 functions definitions

template <typename _Scalar>
SO3<_Scalar>::SO3(const ManifoldDataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
template <typename _OtherDerived>
SO3<_Scalar>::SO3(const ManifoldBase<_OtherDerived>& o)
  : data_(*o.data())
{
  static_assert(Dim == _OtherDerived::Dim,
                "Manifold are not of the same dimension !");
}

template <typename _Scalar>
template <typename _OtherDerived>
SO3<_Scalar>& SO3<_Scalar>::operator=(const ManifoldBase<_OtherDerived>& o)
{
  static_assert(Dim == _OtherDerived::Dim,
                "Manifold are not of the same dimension !");

  data_ = o.data_;
}

template <typename _Scalar>
typename SO3<_Scalar>::ManifoldDataType*
SO3<_Scalar>::data()
{
  return &data_;
}

template <typename _Scalar>
const typename SO3<_Scalar>::ManifoldDataType*
SO3<_Scalar>::data() const
{
  return &data_;
}

template <typename _Scalar>
void SO3<_Scalar>::identity()
{
  std::cout << "SO3 identity\n";
  data_.setIdentity();
}

template <typename _Scalar>
void SO3<_Scalar>::random()
{
  std::cout << "SO3 random\n";
  data_.setRandom();
}

template <typename _Scalar>
SO3<_Scalar> SO3<_Scalar>::inverse() const
{
  std::cout << "SO3 inverse\n";
  return SO3();
}

template <typename _Scalar>
SO3<_Scalar> SO3<_Scalar>::rplus(const SO3::Tangent& /*t*/) const
{
  std::cout << "SO3 rplus\n";
  return Manifold();
}

template <typename _Scalar>
SO3<_Scalar> SO3<_Scalar>::lplus(const SO3::Tangent& /*t*/) const
{
  std::cout << "SO3 lplus\n";
  return Manifold();
}

template <typename _Scalar>
SO3<_Scalar> SO3<_Scalar>::rminus(const SO3& /*m*/) const
{
  std::cout << "SO3 rminus\n";
  return SO3();
}

template <typename _Scalar>
SO3<_Scalar> SO3<_Scalar>::lminus(const SO3& /*m*/) const
{
  std::cout << "SO3 lminus\n";
  return SO3();
}

template <typename _Scalar>
typename SO3<_Scalar>::Tangent SO3<_Scalar>::lift() const
{
  std::cout << "SO3 lift\n";
  return Tangent();
}

template <typename _Scalar>
SO3<_Scalar> SO3<_Scalar>::compose(const SO3& /*m*/) const
{
  std::cout << "SO3 compose\n";
  return SO3();
}

///

template <typename _Scalar>
void SO3<_Scalar>::inverse(
    SO3<_Scalar>& /*m*/, typename SO3<_Scalar>::JacobianType& /*j*/) const
{
  std::cout << "SO3 inverseWithJacobian\n";
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3_H_ */
