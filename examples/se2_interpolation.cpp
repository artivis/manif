#include "manif/SE2.h"
#include "manif/interpolation.h"

#include <iostream>

int main(int argc, char** argv)
{
  manif::INTERP_METHOD interp_method =
      manif::INTERP_METHOD::SLERP;

  if (argc >= 2)
  {
    int selected = atoi(argv[1]);

    switch (selected) {
    case 0:
      interp_method = manif::INTERP_METHOD::SLERP;
      break;
    case 1:
      interp_method = manif::INTERP_METHOD::CUBIC;
      break;
    case 2:
      interp_method = manif::INTERP_METHOD::TWOSTEPS;
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

  double n_pts = 10;

  if (argc >= 4)
  {
    n_pts = atof(argv[3]);
  }

  std::cout << n_k_pts << ","
            << n_pts << ","
            << 0   << "\n";

  // Generate some k points on 8-shaped curve

  std::vector<manif::SE2d> states;
  states.reserve(n_k_pts);
  double t = 0;
  for (double i=0; i<n_k_pts; ++i)
  {
    const double x = std::cos(t);
    const double y = std::sin(2*t) / 2;
    states.emplace_back(manif::SE2d(x,y,std::sin(M_PI*2.)));

    t += M_PI*2. / n_k_pts;

    std::cout << x << ","
              << y << ","
              << 0 << "\n";
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

    for (int j=1; j<n_pts; ++j)
    {
      interp = interpolate(s0, s1,
                           static_cast<double>(j)/n_pts,
                           interp_method);

      std::cout << interp.x() << ","
                << interp.y() << ","
                << interp.angle() << "\n";
    }

    std::cout << s1.x() << ","
              << s1.y() << ","
              << s1.angle() << "\n";
  }

//  manif::SE2d state(0,0,M_PI/2.),
//              state_other(2,0,0);

//  manif::SE2d interp;

//  // Initial Point

//  interp = interpolate(state, state_other,
//                       0, interp_method);

//  std::cout << interp.x() << ","
//            << interp.y() << ","
//            << interp.angle() << "\n";

//  // Interpolated Points

//  for (double i=1; i<n_pts; ++i)
//  {
//    interp = interpolate(state, state_other,
//                         i/n_pts, interp_method);

//    std::cout << interp.x() << ","
//              << interp.y() << ","
//              << interp.angle() << "\n";
//  }

//  // Final Point

//  interp = interpolate(state, state_other,
//                       1, interp_method);

//  std::cout << interp.x() << ","
//            << interp.y() << ","
//            << interp.angle() << "\n";

  return 0;
}
