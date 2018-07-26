#include "manif/SE2.h"
#include "manif/algorithms/interpolation.h"

#include <vector>
#include <iostream>

void heightShapeSmooth(const double degree,
                       const double n_k_pts,
                       const double n_pts)
{
  std::cout << n_k_pts << ","
            << n_pts   << ","
            << degree  << std::endl;

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
  }

  for (const auto& p : states)
    std::cout << p.x() << ","
              << p.y() << ","
              << p.angle() << "\n";

  manif::SE2Tangentd ta = manif::SE2Tangentd::Zero();
  manif::SE2Tangentd tb = manif::SE2Tangentd::Zero();

  std::vector<manif::SE2d> curve;
  for (std::size_t i = 0; i<states.size()-1; ++i)
  {
    const manif::SE2d& ma = states[i];
    const manif::SE2d& mb = states[i+1];

    for (int t=1; t<=n_pts; ++t)
    {
      // t in [0,1]
      const double t_01 = static_cast<double>(t)/(n_pts);

//      std::cout << "at t : " << t_01
//                << " ta = " << ta
//                << " tb = " << tb
//                << "\n";

      const auto mc = interpolate_smooth(ma, mb,
                                         t_01, degree,
                                         ta, tb);

      curve.push_back(mc);
    }
  }

  // Close the loop

  const manif::SE2d& ma = states.back();
  const manif::SE2d& mb = states.front();

  for (int t=1; t<=n_pts; ++t)
  {
    // t in [0,1]
    const double t_01 = static_cast<double>(t)/(n_pts);

    const auto mc = interpolate_smooth(ma, mb, t_01, degree,
                                       manif::SE2Tangentd::Zero(),
                                       manif::SE2Tangentd::Zero());

    curve.push_back(mc);
  }

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

  heightShapeSmooth(degree, n_k_pts, n_pts);

  return 0;
}
