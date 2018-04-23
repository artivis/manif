#ifndef _MANIF_MANIF_SO2_H_
#define _MANIF_MANIF_SO2_H_

#include "manif/SO2_base.h"

#include <Eigen/Core>

#include <iostream>

namespace manif
{

// Forward declare for type traits specialization

template <typename _Scalar> struct SO2;
template <typename _Scalar> struct SO2Tangent;

namespace internal
{

// Traits specialization

template <>
template <typename _Scalar>
struct traits<SO2<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SO2<_Scalar>;
  using Tangent  = SO2Tangent<_Scalar>;

  using Base = SO2Base<SO2<_Scalar>>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int RepSize = 2;

  using ManifoldDataType = Eigen::Matrix<Scalar, RepSize, 1>;
  using TangentDataType  = Eigen::Matrix<Scalar, DoF, 1>;

  using JacobianType = Eigen::Matrix<Scalar, RepSize, RepSize>;
  /*
  using JacobianType0    = Eigen::Matrix<Scalar, RepSize, RepSize>;
  using JacobianType1    = Eigen::Matrix<Scalar, RepSize, DoF>;
  using JacobianType2    = Eigen::Matrix<Scalar, DoF, RepSize>;
  */
};

template <typename _Scalar>
struct traits<SO2Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SO2<_Scalar>;
  using Tangent = SO2Tangent<_Scalar>;

  using Base = SO2Base<SO2<_Scalar>>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

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
struct SO2Tangent : SO2TangentBase<SO2Tangent<_Scalar>>
{
  SO2Tangent() = default;

  template <typename _OtherDerived>
  SO2Tangent(const ManifoldBase<_OtherDerived>& o);

//  SO2Tangent& operator=(const SO2& o);
//  SO2Tangent& operator=(SO2&& o);

  template <typename _OtherDerived>
  SO2Tangent& operator=(const ManifoldBase<_OtherDerived>& o);

  void zero();
  void random();
  SO2<_Scalar> retract() const;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SO2Tangent);

template <typename _Scalar>
template <typename _OtherDerived>
SO2Tangent<_Scalar>::SO2Tangent(const ManifoldBase<_OtherDerived>& o)
{

}

template <typename _Scalar>
template <typename _OtherDerived>
SO2Tangent<_Scalar>& SO2Tangent<_Scalar>::operator=(const ManifoldBase<_OtherDerived>& o)
{

}

template <typename _Scalar>
void SO2Tangent<_Scalar>::zero()
{
  std::cout << "SO2Tangent zero\n";
}

template <typename _Scalar>
void SO2Tangent<_Scalar>::random()
{
  std::cout << "SO2Tangent random\n";
}

template <typename _Scalar>
SO2<_Scalar>
SO2Tangent<_Scalar>::retract() const
{
  std::cout << "SO2Tangent rectract\n";
  return SO2<_Scalar>();
}

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Scalar>
struct SO2 : SO2Base<SO2<_Scalar>>
{
  using Base = SO2Base<SO2<_Scalar>>;

  using Tangent = typename Base::Tangent;

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  using ManifoldDataType2 = Eigen::Matrix<_Scalar, RepSize, 1>;
//  using TangentDataType  = Eigen::Matrix<_Scalar, 2, 1>;
//  using JacobianType = Eigen::Matrix<_Scalar, 2, 2>;

  SO2()  = default;
  ~SO2() = default;

//  SO2(const SO2& o);
//  SO2(SO2&& o);

  template <typename _OtherDerived>
  SO2(const ManifoldBase<_OtherDerived>& o);

//  SO2& operator=(const SO2& o);
//  SO2& operator=(SO2&& o);

  template <typename _OtherDerived>
  SO2& operator=(const ManifoldBase<_OtherDerived>& o);

  SO2(const ManifoldDataType& d);

  ManifoldDataType* data();

  const ManifoldDataType* data() const;

  void identity();

  void random();

  SO2 inverse() const;

  SO2 rplus(const SO2::Tangent& /*t*/) const;

  SO2 lplus(const SO2::Tangent& /*t*/) const;

  SO2 rminus(const SO2& /*m*/) const;

  SO2 lminus(const SO2& /*m*/) const;

  Tangent lift() const;

  SO2 compose(const SO2& /*m*/) const;

  ///

  void inverse(SO2& m, JacobianType& j) const;

protected:

  ManifoldDataType data_;
};

MANIF_EXTRA_MANIFOLD_TYPEDEF(SO2)

/// SO2 functions definitions

template <typename _Scalar>
SO2<_Scalar>::SO2(const ManifoldDataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
template <typename _OtherDerived>
SO2<_Scalar>::SO2(const ManifoldBase<_OtherDerived>& o)
  : data_(*o.data())
{
  static_assert(Dim == _OtherDerived::Dim,
                "Manifold are not of the same dimension !");
}

template <typename _Scalar>
template <typename _OtherDerived>
SO2<_Scalar>& SO2<_Scalar>::operator=(const ManifoldBase<_OtherDerived>& o)
{
  static_assert(Dim == _OtherDerived::Dim,
                "Manifold are not of the same dimension !");

  data_ = o.data_;
}

template <typename _Scalar>
typename SO2<_Scalar>::ManifoldDataType*
SO2<_Scalar>::data()
{
  return &data_;
}

template <typename _Scalar>
const typename SO2<_Scalar>::ManifoldDataType*
SO2<_Scalar>::data() const
{
  return &data_;
}

template <typename _Scalar>
void SO2<_Scalar>::identity()
{
  std::cout << "SO2 identity\n";
  data_.setIdentity();
}

template <typename _Scalar>
void SO2<_Scalar>::random()
{
  std::cout << "SO2 random\n";
  data_.setRandom();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::inverse() const
{
  std::cout << "SO2 inverse\n";
  return SO2();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::rplus(const SO2::Tangent& /*t*/) const
{
  std::cout << "SO2 rplus\n";
  return Manifold();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::lplus(const SO2::Tangent& /*t*/) const
{
  std::cout << "SO2 lplus\n";
  return Manifold();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::rminus(const SO2& /*m*/) const
{
  std::cout << "SO2 rminus\n";
  return SO2();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::lminus(const SO2& /*m*/) const
{
  std::cout << "SO2 lminus\n";
  return SO2();
}

template <typename _Scalar>
typename SO2<_Scalar>::Tangent SO2<_Scalar>::lift() const
{
  std::cout << "SO2 lift\n";
  return Tangent();
}

template <typename _Scalar>
SO2<_Scalar> SO2<_Scalar>::compose(const SO2& /*m*/) const
{
  std::cout << "SO2 compose\n";
  return SO2();
}

///

template <typename _Scalar>
void SO2<_Scalar>::inverse(
    SO2<_Scalar>& /*m*/, typename SO2<_Scalar>::JacobianType& /*j*/) const
{
  std::cout << "SO2 inverseWithJacobian\n";
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_H_ */
