# Writing generic code

All Lie group classes defined in `manif` have in common that they inherit from a templated base class.
Therefore, template-based generic code can be written - similarly to Eigen.

## Examples

### Small example

Let us write a simple function that take any group object and prints some information about it,

```cpp
#include <iostream>
#include <manif/manif.h>

using namespace manif;

template <typename Derived>
void print(const LieGroupBase<Derived>& g)
{
  std::cout << "Degrees of freedom: " << g::DoF << "\n"
            << "Underlying representation vector size: " << g::RepSize << "\n"
            << "Current values: " << g << "\n;
}

int main()
{
  SE2d p_2d;
  print(p_2d);

  SE3d p_3d;
  print(p_3d);
}
```

### Multiple templated arguments

Let us write a function that takes two group objects and performs some computation,

```cpp
#include <manif/manif.h>

using namespace manif;

template <typename DerivedA, typename DerivedB>
typename DerivedA::Scalar
ominusSquaredWeightedNorm(
  const LieGroupBase<DerivedA>& state,
  const LieGroupBase<DerivedB>& state_other
)
{
  return (state-state_other).squaredWeightedNorm();
}

int main()
{
  SE2d state = SE2d::Random();

  SE2d::DataType state_other_data = SE2d::DataType::Random();

  Eigen::Map<SE2d> state_other_map(state_other_data.data());

  double osn = ominusSquaredWeightedNorm(state, state_other_map);

  ...
}
```
