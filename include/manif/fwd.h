#ifndef _MANIF_MANIF_FWD_H_
#define _MANIF_MANIF_FWD_H_

namespace manif
{
namespace internal
{

template <typename T> struct traits;

/// @note the following is from the Eigen library

// here we say once and for all that traits<const T> == traits<T>
// When constness must affect traits, it has to be constness on template parameters on which T itself depends.
// For example, traits<Map<const T> > != traits<Map<T> >, but
//              traits<const Map<T> > == traits<Map<T> >
template<typename T> struct traits<const T> : traits<T> {};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_FWD_H_ */

#define DEFINE_MANIFOLD_TANGENT                                 \
  struct Tangent : TangentBase<typename ManifoldBase::Manifold> \
  {                                                             \
    void zero();                                                \
    void random();                                              \
    typename ManifoldBase::Manifold retract() const;            \
  };

#define MANIFOLD_BASE_TYPEDEF \
  using Base::plus; \
  using Base::operator +; \
  using Base::operator +=; \
  using Base::operator *; \
  using Base::operator *=; \
  using Manifold = typename Base::Manifold; \

#define MANIFOLD_TYPEDEF                            \
  using Scalar = _Scalar;                           \
  using Manifold = typename ManifoldBase::Manifold; \
  using Type = Manifold;                            \
  using ManifoldBase::Dim;                          \
  using ManifoldBase::RepSize; \
  using ManifoldBase::plus; \
  using ManifoldBase::operator +; \
  using ManifoldBase::operator +=; \
  using ManifoldBase::operator *; \
  using ManifoldBase::operator *=; \

#define INHERIT_MANIFOLD_OPERATOR \
  using Base::operator =;

#define COMPLETE_MANIFOLD_TYPEDEF \
  DEFINE_MANIFOLD_TANGENT         \
  MANIFOLD_TYPEDEF                \

#define EXTRA_MANIFOLD_TYPEDEF(manifold)                          \
  using manifold##f = manifold<float>;                            \
  using manifold##d = manifold<double>;                           \
                                                                  \
  template <typename _Scalar>                                     \
  using manifold##Tangent = typename manifold<_Scalar>::Tangent;  \
                                                                  \
  using manifold##Tangentf = manifold##Tangent<float>;            \
  using manifold##Tangentd = manifold##Tangent<double>;
