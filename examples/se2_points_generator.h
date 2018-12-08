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
  states.emplace_back(x,y,M_PI/2);

  double t = 0;
  for (unsigned int i=1; i<k; ++i)
  {
    t += M_PI*2. / double(k);

    const double x = std::cos(t);
    const double y = std::sin(2.*t) / 2.;

    const double t = std::atan2(y-states.back().y(),
                                x-states.back().x());

    states.emplace_back(x,y,t);
  }

  return states;
}

} /* namespace manif */
