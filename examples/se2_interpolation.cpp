#include "manif/SE2.h"
#include "manif/algorithms/interpolation.h"

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

void heightShapeInterp(const manif::INTERP_METHOD interp_method,
                       const double n_k_pts,
                       const double n_pts)
{
  std::cout << n_k_pts << ","
            << n_pts << ","
            << 0   << "\n";

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

//  Close the loop

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
  manif::INTERP_METHOD interp_method =
      manif::INTERP_METHOD::SLERP;

  (void)interp_method;

  int selected = 0;

  if (argc >= 2)
  {
    selected = atoi(argv[1]);

    switch (selected) {
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
      interp_method = manif::INTERP_METHOD::SLERP;
      break;
    }
  }

  double n_k_pts = 10;

  if (argc >= 3)
  {
    n_k_pts = atof(argv[2]);
  }
  (void)n_k_pts;

  double n_pts = 10;

  if (argc >= 4)
  {
    n_pts = atof(argv[3]);
  }

//  std::cout << 3 << ","
//            << n_pts << ","
//            << selected << std::endl;

  twoPointsInterp(interp_method, n_pts);
//  fivePointsInterp(interp_method, n_pts);
//  heightShapeInterp(interp_method, n_k_pts, n_pts);

//  heightShapeBezier(5, n_k_pts, n_pts);

//  std::cout << 3 << ","
//            << n_pts << ","
//            << 0   << std::endl;

//  // Generate 3 points
//  std::vector<manif::SE2d> states;
//  states.emplace_back(0,0,-M_PI/2);
//  states.emplace_back(0,2,-M_PI/4);
//  states.emplace_back(2,2,0);

//  for (const auto& p : states)
//    std::cout << p.x() << ","
//              << p.y() << ","
//              << p.angle() << "\n";

////  const auto curve = computeBezierCurve(states, 3, n_pts);
//  const auto curve = computeBezierCurve(states, 3, n_pts);

//  for (const auto& p : curve)
//    std::cout << p.x() << ","
//              << p.y() << ","
//              << p.angle() << "\n";

  return 0;
}
