#include "manif/SE2.h"
#include <vector>

namespace manif {

/**
 * @brief Generate k SE2 points on an height shape curve.
 * @param[in] k Number of points to generate.
 * @return vector of k points.
 */
std::vector<manif::SE2d>
generateSE2PointsOnHeightShape(const unsigned int k)
{
  // Generate some k points on 8-shaped curve
  std::vector<manif::SE2d> states;
  states.reserve(k);

  const double x = std::cos(0);
  const double y = std::sin(0)/2;
  states.emplace_back(x,y,MANIF_PI/2);

  double t = 0;
  for (unsigned int i=1; i<k; ++i)
  {
    t += MANIF_PI*2. / double(k);

    const double xi = std::cos(t);
    const double yi = std::sin(2.*t) / 2.;

    const double thi = std::atan2(yi-states.back().y(),
                                  xi-states.back().x());

    states.emplace_back(xi,yi,thi);
  }

  return states;
}

} /* namespace manif */
