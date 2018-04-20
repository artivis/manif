#include "manif/SO2.h"

using namespace manif;

template <typename Derived>
void setIdentity(ManifoldBase<Derived>& m)
{
  m.identity();
}

int main() {

  std::cout << "Starting dummy.\n";

  SO2d so2 = SO2d::Identity();

  so2.identity();

  auto l = so2.lift();

  auto r = l.retract();

  setIdentity(r);

  return 0;
}
