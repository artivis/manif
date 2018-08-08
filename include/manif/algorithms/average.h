#ifndef _MANIF_MANIF_AVERAGE_H_
#define _MANIF_MANIF_AVERAGE_H_

#include "manif/impl/manifold_base.h"
//#include "manif/interpolation.h"

namespace manif
{

//template <template <typename Manifold, typename...Args> class Container,
//          typename Manifold, typename...Args>
//Manifold
//average_slerp(const Container<Manifold, Args...>& mans)
//{
//  if (mans.empty())
//    return Manifold();
//  else if (mans.size() == 1)
//    return *mans.begin();

//  auto it = mans.begin();

//  Manifold carry = *it;

//  ++it;
//  double i = 2;
//  for (; it != mans.end(); ++it, ++i)
//  {
//    carry = interpolate(carry, *it, (i-1.)/i);
//  }

//  return carry;
//}

/**
 * @note see
 * "Bi-invariant Means in Lie Groups.
 * Application to Left-in variant Polyaffine Transformations" p. 21 Sec. 4.2
 * @link ftp://ftp-sop.inria.fr/epidaure/Publications/Arsigny/arsigny_rr_biinvariant_mean.pdf
 */
template <template <typename Manifold, typename...Args> class Container,
          typename Manifold, typename...Args>
Manifold
average_biinvariant(const Container<Manifold, Args...>& mans,
                    int max_iterations = 20)
{
  using Scalar  = typename Manifold::Scalar;
  using Tangent = typename Manifold::Tangent;

  if (mans.empty())
    return Manifold();
  else if (mans.size() == 1)
    return *mans.begin();

  auto m0 = *mans.begin();
  Manifold avg;

  const Scalar w = Scalar(1./mans.size());

  for (int i=0; i<max_iterations; ++i)
  {
    auto it = mans.begin();
    const auto end = mans.end();

    Tangent ts = Tangent::Zero();
    for (; it != end; ++it)
    {
      ts.coeffs() += w * m0.between(*it).lift().coeffs();
    }

    auto avg = m0.rplus(ts);

    if (avg.between(m0).lift().coeffs().squaredNorm() < Constants<Scalar>::eps_s)
      return avg;

    m0 = avg;
  }

  return avg;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_AVERAGE_H_ */
