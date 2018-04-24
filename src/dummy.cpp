#include "manif/SO2.h"
#include "manif/SO3.h"

using namespace manif;

template <typename Derived>
void setIdentity(ManifoldBase<Derived>& m)
{
  std::cout << "Setting Identity\n";
  m.identity();
}

template <typename Derived>
void setRandom(ManifoldBase<Derived>& m)
{
  std::cout << "Setting Random\n";
  m.random();
}

template <typename Derived>
void computeInverseJac(ManifoldBase<Derived>& m)
{
  std::cout << "Compute inverse with Jac\n";

  using T = typename ManifoldBase<Derived>::Scalar;

  SO2<T> minv = m;
  typename SO2<T>::JacobianMtoM Jac;
  m.inverse(minv, Jac);
}

template <typename ManDerived, typename TanDerived>
void doPlus(const ManifoldBase<ManDerived>& m, const TangentBase<TanDerived>& t)
{
  auto ret = m.plus(t);
}

int main()
{
  static_assert(std::is_same<SO2<double>, SO2d>::value, "ok");
  static_assert(std::is_same<SO2<float>,  SO2f>::value, "ok");

  static_assert(std::is_same<SO2d, SO2d::Manifold>::value, "ok");

  static_assert(std::is_same<SO2d::Tangent, SO2Tangentd>::value, "ok");
  static_assert(std::is_same<SO2f::Tangent, SO2Tangentf>::value, "ok");

  std::cout << "Starting dummy.\n\n";

  std::cout << "SO2d::Dim : " << SO2d::Dim << "\n";
  std::cout << "SO2d::RepSize : " << SO2d::RepSize << "\n";

  std::cout << "SO2d::Tangent::Dim : " << SO2d::Tangent::Dim << "\n";
  std::cout << "SO2d::Tangent::RepSize : " << SO2d::Tangent::RepSize << "\n";

  SO2d so2 = SO2d::Random();

  so2 = SO2d::Identity();

  so2.identity();

  SO2d so2_inv = so2.inverse();

  auto so2_lift = so2.lift();

  static_assert(std::is_same<decltype(so2_lift), SO2d::Tangent>::value, "ok");

  auto so2_retract = so2_lift.retract();

  static_assert(std::is_same<decltype(so2_retract), SO2d>::value, "ok");

  auto so2_rplus = so2.rplus( so2_lift );

  static_assert(std::is_same<decltype(so2_rplus), SO2d>::value, "ok");

  so2_rplus = so2 + so2_lift;
  so2_rplus += so2_lift;

  so2_rplus = so2.plus( so2_lift );

  doPlus(so2, so2_lift);

  // Should **not** compile
//  doPlus(so2, SO3Tangentd{});

  auto so2_lplus = so2.lplus( so2_lift );

  static_assert(std::is_same<decltype(so2_lplus), SO2d>::value, "ok");

  auto so2_rminus = so2.rminus(so2_inv);

//  so2_rplus = so2 - so2_lift;
//  so2_rplus -= so2_lift;

//  so2_rplus = so2.minus( so2_inv );

  static_assert(std::is_same<decltype(so2_rminus), SO2d>::value, "ok");

  auto so2_lminus = so2.lminus(so2_inv);

  static_assert(std::is_same<decltype(so2_lminus), SO2d>::value, "ok");

  auto so2_compose = so2.compose(so2_inv);

  static_assert(std::is_same<decltype(so2_compose), SO2d>::value, "ok");

  so2_compose = so2 * so2_inv;

  so2_compose = so2 * so2_inv;
  so2_compose *= so2_inv;

  SO2d so2_between = so2.between(so2_inv);

  static_assert(std::is_same<decltype(so2_between), SO2d>::value, "ok");

  setIdentity(so2);

  setRandom(so2);

  //////

  SO2Tangentd so2_tan(0.17);
  so2_tan = SO2Tangentd(SO2Tangentd::TangentDataType(0.17));

  so2_tan.zero();


  //////

  SO2d::JacobianMtoM so2_inv_jac;

  so2.inverse(so2_inv, so2_inv_jac);

  computeInverseJac(so2);

  std::cout << "\n";

  return 0;
}
