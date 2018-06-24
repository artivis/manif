#include "manif/SE2.h"
#include "manif/algorithms/decasteljau.h"

#include <iostream>

void heightShapeDeCasteljau(const double degree,
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


  const auto curve = decasteljau(states, degree, n_pts);

  for (const auto& p : curve)
    std::cout << p.x() << ","
              << p.y() << ","
              << p.angle() << "\n";
}

int main(int argc, char** argv)
{
  double n_k_pts = 10;

  if (argc >= 2)
  {
    n_k_pts = atof(argv[1]);
  }

  double n_pts = 10;

  if (argc >= 3)
  {
    n_pts = atof(argv[2]);
  }

  double degree = 3;

  if (argc >= 4)
  {
    degree = atof(argv[3]);
  }

  heightShapeDeCasteljau(degree, n_k_pts, n_pts);

  return 0;
}
