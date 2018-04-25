#ifndef _MANIF_MANIF_SO2TANGENT_MAP_H_
#define _MANIF_MANIF_SO2TANGENT_MAP_H_

#include "manif/impl/SO2Tangent.h"

namespace Eigen
{

/// @todo

/*
template <class _Scalar>
class Map<manif::SO2<_Scalar>, 0>
    : public manif::SO2Base<Map<manif::SO2<_Scalar>, 0> >
{
  using Base = manif::SO2Base<Map<manif::SO2<_Scalar>, 0> >;

public:

  using Tangent = typename Base::Tangent;

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

//  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)

  //  using Base::data;
  using Base::matrix;
  using Base::rotation;
  using Base::identity;
  using Base::random;
  using Base::inverse;
  using Base::rplus;
  using Base::lplus;
  using Base::rminus;
  using Base::lminus;
  using Base::lift;
  using Base::compose;

  Map(Scalar* coeffs) : data_(coeffs) { }

  template <typename _Derived>
  Manifold& operator =(const manif::SO2Base<_Derived>& o)
  {
    data_ = *o.data();
    return *this;
  }

  ManifoldDataType const* data() const { return &data_; }

protected:

  friend class manif::ManifoldBase<Map<manif::SO2<_Scalar>, 0>>;

  ManifoldDataType* data() { return &data_; }
  ManifoldDataType data_;
};

template <class _Scalar>
class Map<const manif::SO2<_Scalar>, 0>
    : public manif::SO2Base<Map<const manif::SO2<_Scalar>, 0> >
{
  using Base = manif::SO2Base<Map<const manif::SO2<_Scalar>, 0> >;

public:

  using Tangent = typename Base::Tangent;

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

//  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)

  //  using Base::data;
  using Base::matrix;
  using Base::rotation;
  using Base::identity;
  using Base::random;
  using Base::inverse;
  using Base::rplus;
  using Base::lplus;
  using Base::rminus;
  using Base::lminus;
  using Base::lift;
  using Base::compose;

  Map(Scalar* coeffs) : data_(coeffs) { }

  ManifoldDataType const* data() const { return &data_; }

protected:

  friend class manif::ManifoldBase<Map<manif::SO2<_Scalar>, 0>>;

  ManifoldDataType* data() { return &data_; }
  ManifoldDataType data_;
};
*/

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SO2TANGENT_MAP_H_ */
