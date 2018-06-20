#include "manif/SE2.h"
#include "manif/interpolation.h"

#include <iostream>

int main(int argc, char** argv)
{
  manif::INTERP_METHOD interp_method =
      manif::INTERP_METHOD::SLERP;

  if (argc == 2)
  {
    int selected = atoi(argv[1]);

    switch (selected) {
    case 0:
      interp_method = manif::INTERP_METHOD::SLERP;
      break;
    case 1:
      interp_method = manif::INTERP_METHOD::CUBIC;
      break;
    default:
      interp_method = manif::INTERP_METHOD::SLERP;
      break;
    }
  }

  manif::SE2d state(0,0,M_PI/2.),
              state_other(2,0,0);

  manif::SE2d interp;

  double n_pts = 10;

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

  return 0;
}
