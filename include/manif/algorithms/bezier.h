#ifndef _MANIF_MANIF_BEZIER_H_
#define _MANIF_MANIF_BEZIER_H_

#include "manif/impl/lie_group_base.h"
#include "manif/algorithms/interpolation.h"

#include <vector>

namespace manif {

/**
 * @brief Piece-wise Bezier curve fitting using Bernstein polynomials on Lie groups.
 * Each piece is defined by a polynomial of degree 'degree'
 * thus by 'degree+1' control points.
 *
 * @note The knots are not smoothed in any ways!
 *
 * @param control_points, a set of control points.
 * @param degree, the smoothness degree of the fitted curve's pieces.
 * @param k_interp, the number of points to interpolate per pieces.
 * @return The fitted Piece-wise Bezier curve.
 *
 * @see decasteljau
 */
template <typename LieGroup>
std::vector<typename LieGroup::LieGroup>
bezier(
  const std::vector<LieGroup>& control_points,
  const unsigned int degree,
  const unsigned int k_interp
)
{
  MANIF_CHECK(
    control_points.size() > 2,
    "Input trajectory must have more than two points!"
  );
  MANIF_CHECK(
    degree <= control_points.size(),
    "Degree must be less or equal to the number of input points!"
  );
  MANIF_CHECK(
    k_interp > 0,
    "k_interp must be greater than zero!"
  );

  const unsigned int num_ctrl_pts = degree + 1;

  const unsigned int n_segments = static_cast<unsigned int>(
    std::floor(
      double(num_ctrl_pts * control_points.size() + control_points.size()) /
      double(num_ctrl_pts * num_ctrl_pts)
    )
  );

  std::vector<std::vector<const LieGroup*>> segments_control_points(
    n_segments,
    std::vector<const LieGroup*>(num_ctrl_pts)
  );
  for (unsigned int t = 0; t < n_segments; ++t)
  {
    // Retrieve control points of the current segment
    for (int n = 0; n < num_ctrl_pts; ++n)
    {
      segments_control_points[t][n] = &control_points[t * (num_ctrl_pts - 1) + n];
    }
  }

  std::vector<LieGroup> curve;
  curve.reserve(segments_control_points.size()*k_interp);
  curve.push_back(control_points[0]);
  LieGroup Qc;
  for (unsigned int s=0; s<segments_control_points.size(); ++s)
  {
    for (unsigned int t=1; t<=k_interp; ++t)
    {
      LieGroup Qc = *segments_control_points[s][0];

      // recursive chunk of the algo, compute tmp control points.
      for (unsigned int i=1; i<num_ctrl_pts; ++i)
      {
        Qc += (
          polynomialBernstein(
            double(degree),
            double(i),
            double(t)/k_interp // t in ]0,1]
          ) * (
            *segments_control_points[s][i] - *segments_control_points[s][0]
          )

        );
      }
      curve.push_back(Qc);
    }
  }

  return curve;
}

} // namespace manif

#endif // _MANIF_MANIF_BEZIER_H_
