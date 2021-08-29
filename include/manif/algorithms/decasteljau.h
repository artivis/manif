#ifndef _MANIF_MANIF_DECASTELJAU_H_
#define _MANIF_MANIF_DECASTELJAU_H_

#include "manif/impl/lie_group_base.h"
#include "manif/algorithms/interpolation.h"

#include <vector>

namespace manif {

/**
 * @brief Piece-wise Bezier curve fitting using the DeCasteljau algorithm
 * on Lie groups.
 * Each piece is geometrically and recursively constructed
 * from 'degree+1' control points.
 *
 * @note The knots are not smoothed in any ways!
 *
 * @param control_points, a set of control points.
 * @param degree, the smoothness degree of the fitted curve's pieces.
 * @param k_interp, the number of points to interpolate per pieces.
 * @return The fitted Piece-wise Bezier curve.
 *
 * @note A naive implementation of the DeCasteljau algorithm on Lie groups.
 *
 * @link https://www.wikiwand.com/en/De_Casteljau%27s_algorithm
 *
 * @see bezier
 */
template <typename LieGroup>
std::vector<typename LieGroup::LieGroup>
decasteljau(
  const std::vector<LieGroup>& control_points,
  const unsigned int degree,
  const unsigned int k_interp,
  const INTERP_METHOD interp = INTERP_METHOD::SLERP
)
{
  MANIF_CHECK(
    control_points.size() > 2,
    "Input control_points must have more than two points!"
  );
  MANIF_CHECK(
    degree < control_points.size(),
    "Degree must be lower than the number of input points!"
  );
  MANIF_CHECK(
    k_interp > 0,
    "k_interp must be greater than zero!"
  );

  const unsigned int num_ctrl_pts = degree + 1;

  // Number of connected, non-overlapping segments
  const unsigned int n_segments = static_cast<unsigned int>(
    std::floor(
      double(num_ctrl_pts*control_points.size()+control_points.size()) /
      double(num_ctrl_pts*num_ctrl_pts)
    )
  );

  std::vector<std::vector<const LieGroup*>> segments_control_points(
    n_segments,
    std::vector<const LieGroup*>(num_ctrl_pts)
  );
  for (unsigned int t = 0; t < n_segments; ++t)
  {
    // Retrieve control points of the current segment
    for (unsigned int n = 0; n < num_ctrl_pts; ++n)
    {
      segments_control_points[t][n] = &control_points[t * (num_ctrl_pts - 1) + n];
    }
  }

  // Actual curve fitting
  std::vector<LieGroup> curve;
  curve.reserve(segments_control_points.size()*k_interp);
  curve.push_back(control_points[0]);
  for (unsigned int s=0; s<segments_control_points.size(); ++s)
  {
    for (unsigned int t=1; t<=k_interp; ++t)
    {
      // t in ]0,1]
      const double t_01 = double(t)/k_interp;

      std::vector<LieGroup> Qs, Qs_tmp;

      for (const auto m : segments_control_points[s]) {
        Qs.emplace_back(*m);
      }

      // recursive chunk of the algo, compute tmp control points.
      for (unsigned int i=0; i<degree; ++i)
      {
        for (unsigned int q=0; q<Qs.size()-1; ++q)
        {
          Qs_tmp.emplace_back( interpolate(Qs[q], Qs[q+1], t_01, interp) );
        }

        Qs = Qs_tmp;
        Qs_tmp.clear();
      }

      curve.emplace_back(std::move(Qs[0]));
    }
  }

  return curve;
}

} // namespace manif

#endif // _MANIF_MANIF_DECASTELJAU_H_
