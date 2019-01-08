#include "manif/SE2.h"
#include "manif/algorithms/interpolation.h"
#include "se2_points_generator.h"

#include <vector>
#include <iostream>

void twoPointsInterp(const manif::INTERP_METHOD interp_method,
                     const double n_pts)
{
  std::cout << 2 << ","
            << n_pts << ","
            << static_cast<typename std::underlying_type<manif::INTERP_METHOD>::type>(interp_method)   << "\n";

  manif::SE2d state(0,0,M_PI/2.),
              state_other(2,0,0);

  std::cout << state.x() << ","
            << state.y() << ","
            << state.angle() << "\n";

  std::cout << state_other.x() << ","
            << state_other.y() << ","
            << state_other.angle() << "\n";

  manif::SE2d interp;

  // Initial Point

  interp = interpolate(state, state_other,
                       0, interp_method);

  std::cout << interp.x() << ","
            << interp.y() << ","
            << interp.angle() << "\n";

  // Interpolated Points

  for (double i=1; i<n_pts; ++i)
  {
    interp = interpolate(state, state_other,
                         i/n_pts, interp_method);

    std::cout << interp.x() << ","
              << interp.y() << ","
              << interp.angle() << "\n";
  }

  // Final Point

  interp = interpolate(state, state_other,
                       1, interp_method);

  std::cout << interp.x() << ","
            << interp.y() << ","
            << interp.angle() << "\n";
}

void fivePointsInterp(const manif::INTERP_METHOD interp_method,
                      const double n_pts)
{
  std::cout << 5 << "," << n_pts << "," << 0 << "\n";

  std::vector<manif::SE2d> states;
  states.reserve(5);

  states.emplace_back(0  ,0  , M_PI/2.);
  states.emplace_back(2.5,3.5,-M_PI/4.);
  states.emplace_back(4  ,2.5, 0);
  states.emplace_back(9  ,  1,-M_PI/2.);
  states.emplace_back(6  , -1,-M_PI);

  for (const auto& p : states)
    std::cout << p.x() << ","
              << p.y() << ","
              << p.angle() << "\n";

  // Interpolate between k-points

  manif::SE2d interp;

  for (int i=0; i<states.size()-1; ++i)
  {
    const manif::SE2d& s0 = states[i];
    const manif::SE2d& s1 = states[i+1];

    std::cout << s0.x() << ","
              << s0.y() << ","
              << s0.angle() << "\n";

    for (int j=1; j<=n_pts; ++j)
    {
      interp = interpolate(s0, s1,
                           static_cast<double>(j)/(n_pts+1),
                           interp_method);

      std::cout << interp.x() << ","
                << interp.y() << ","
                << interp.angle() << "\n";
    }

    std::cout << s1.x() << ","
              << s1.y() << ","
              << s1.angle() << "\n";
  }

  // Close the loop

  const manif::SE2d& s0 = states.back();
  const manif::SE2d& s1 = states[0];

  std::cout << s0.x() << ","
            << s0.y() << ","
            << s0.angle() << "\n";

  for (int j=1; j<=n_pts; ++j)
  {
    interp = interpolate(s0, s1,
                         static_cast<double>(j)/(n_pts+1),
                         interp_method);

    std::cout << interp.x() << ","
              << interp.y() << ","
              << interp.angle() << "\n";
  }

  std::cout << s1.x() << ","
            << s1.y() << ","
            << s1.angle() << "\n";
}

/*
void heightShapeBezier(const double degree,
                       const double n_k_pts,
                       const double n_pts)
{
  std::cout << n_k_pts << ","
            << n_pts << ","
            << 0   << std::endl;

  // Generate some k points on 8-shaped curve
  std::vector<manif::SE2d> states;
  states.reserve(n_k_pts);

  const double x = std::cos(0);
  const double y = std::sin(0)/2;
  states.emplace_back(x,y,M_PI/2);

  double t = 0;
  for (double i=1; i<n_k_pts; ++i)
  {
    t += M_PI*2. / n_k_pts;

    const double x = std::cos(t);
    const double y = std::sin(2*t) / 2;

    const double t = std::atan2(y-states.back().y(),
                                x-states.back().x());

    states.emplace_back(x,y,t);

    std::cout << x << ","
              << y << ","
              << t << "\n";
  }


  const auto curve = computeBezierCurve(states, degree, n_pts);

  for (const auto& p : curve)
    std::cout << p.x() << ","
              << p.y() << ","
              << p.angle() << "\n";
}
*/

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: .se2_interpolation <k> <i> <p>\n";
    std::cout << "\t with k: number of initial points on the 8-shaped curve.\n";
    std::cout << "\t with i: interpolation algorithm to use.\n";
    std::cout << "\t with p: number of points to generate between consecutive points of the initial curve.\n";
    std::cout << "\t Interpolation algorithm are : 0-Slerp / 1-Cubic / 2-CN-Smooth.\n";
    return EXIT_SUCCESS;
  }

  int k, i, p;

  k = atoi(argv[1]);
  i = atoi(argv[2]);
  p = atoi(argv[3]);

  manif::INTERP_METHOD interp_method;
  switch (i) {
  case 0:
    interp_method = manif::INTERP_METHOD::SLERP;
    break;
  case 1:
    interp_method = manif::INTERP_METHOD::CUBIC;
    break;
  case 2:
    interp_method = manif::INTERP_METHOD::CNSMOOTH;
    break;
  default:
    std::cerr << "Interpolation method 'i' must be in [0,2] !\n";
    return EXIT_FAILURE;
    break;
  }

  const auto points = manif::generateSE2PointsOnHeightShape(k);

  // Interpolate between k-points
  // between each consecutive points
  // of the initial curve.

  std::vector<manif::SE2d> interpolated;

  // Initial point with Tangent t0 = 0

  manif::SE2Tangentd t0 = manif::SE2Tangentd::Zero();
  manif::SE2Tangentd t1 = points[1].lminus(points[0]);

  for (int j=1; j<=p; ++j)
  {
    interpolated.push_back(
      interpolate(points[0], points[1],
                  double(j)/double(p+1),
                  interp_method
                  , t0, t1
        )
    );
  }

  // Intermediate points

  for (int i=1; i<points.size()-1; ++i)
  {
    const manif::SE2d& s0 = points[ i ];
    const manif::SE2d& s1 = points[i+1];

//    t0 = points[ i ] - points[i-1];
//    t1 = points[i+1] - points[ i ];

    t0 = points[ i ].lminus(points[i-1]);
    t1 = points[i+1].lminus(points[ i ]);

    for (int j=1; j<=p; ++j)
    {
      interpolated.push_back(
        interpolate(s0, s1,
                    static_cast<double>(j)/(p+1),
                    interp_method
                    , t0, t1
        )
      );
    }
  }

  // Close the loop
  // Final point with Tangent t1 = 0

  const manif::SE2d& s0 = points.back();
  const manif::SE2d& s1 = points[0];

//  t0 = points.back() - points[points.size()-2];
//  t1 = manif::SE2Tangentd::Zero();

  t0 = points.back().lminus(points[points.size()-2]);
  t1 = manif::SE2Tangentd::Zero();

  for (int j=1; j<=p; ++j)
  {
    interpolated.push_back(
      interpolate(s0, s1,
                  static_cast<double>(j)/(p+1),
                  interp_method
                  , t0, t1
      )
    );
  }

  // Print in terminal

  std::cout << k << ", " << i << ", " << p << "\n";

  for (const auto& point : points)
  {
    std::cout << point.x() << ","
              << point.y() << ","
              << point.angle() << "\n";
  }

  for (const auto& interp : interpolated)
  {
    std::cout << interp.x() << ","
              << interp.y() << ","
              << interp.angle() << "\n";
  }

  return EXIT_SUCCESS;
}
