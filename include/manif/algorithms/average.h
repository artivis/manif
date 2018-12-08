#ifndef _MANIF_MANIF_AVERAGE_H_
#define _MANIF_MANIF_AVERAGE_H_

#include "manif/impl/lie_group_base.h"
//#include "manif/interpolation.h"

namespace manif {

//template <template <typename LieGroup, typename...Args> class Container,
//          typename LieGroup, typename...Args>
//LieGroup
//average_slerp(const Container<LieGroup, Args...>& mans)
//{
//  if (mans.empty())
//    return LieGroup();
//  else if (mans.size() == 1)
//    return *mans.begin();

//  auto it = mans.begin();

//  LieGroup carry = *it;

//  ++it;
//  double i = 2;
//  for (; it != mans.end(); ++it, ++i)
//  {
//    carry = interpolate(carry, *it, (i-1.)/i);
//  }

//  return carry;
//}

/**
 * @brief Compute an average point on Lie groups given a list of
 * 'close' points.
 * @param[in] points A list of 'close' points to compute an average point from.
 * @return The average point of the input points.
 *
 * @note see (a)
 * "Bi-invariant Means in Lie Groups.
 * Application to Left-in variant Polyaffine Transformations" p. 21 Sec. 4.2
 * @link ftp://ftp-sop.inria.fr/epidaure/Publications/Arsigny/arsigny_rr_biinvariant_mean.pdf
 *
 * @note see also (b)
 * "A globally convergent numerical algorithm for computing the center of
 * mass on compact Lie groups."
 * @link http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.721.8010&rep=rep1&type=pdf
 */
template <template <typename LieGroup, typename...Args> class Container,
          typename LieGroup, typename...Args>
LieGroup
average_biinvariant(const Container<LieGroup, Args...>& points,
                    int max_iterations = 20)
{
  using Scalar  = typename LieGroup::Scalar;
  using Tangent = typename LieGroup::Tangent;

  if (points.empty())
    return LieGroup();
  else if (points.size() == 1)
    return *points.begin();

  auto m0 = *points.begin();
  LieGroup avg;

  const Scalar w = Scalar(1) / Scalar(points.size());

  Tangent ts;
  for (int i=0; i<max_iterations; ++i)
  {
    auto it        = points.begin();
    const auto end = points.end();

    ts = Tangent::Zero();
    for (; it != end; ++it)
    {
      ts.coeffs() += w * m0.between(*it).lift().coeffs();
    }

    // This stopping criterion is from (b)
//    if (ts.coeffs().squaredNorm() < Constants<Scalar>::eps_s)
//      return avg;

    // This stopping criterion is from (a)
    avg = m0.rplus(ts);

    if (avg.between(m0).lift().coeffs().squaredNorm() < Constants<Scalar>::eps_s)
      return avg;

    m0 = avg;
  }

  return avg;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_AVERAGE_H_ */
