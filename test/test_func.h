#ifndef _MANIF_MANIF_TEST_FUNCTIONS_H_
#define _MANIF_MANIF_TEST_FUNCTIONS_H_

#include "manif/impl/lie_group_base.h"

namespace manif {

template <typename _Derived, typename _Other>
void copy_assign(LieGroupBase<_Derived>& state,
                 const _Other& state_other)
{
  state = state_other;
}

template <typename _Derived, typename _DerivedOther>
void copy_assign_base(LieGroupBase<_Derived>& state,
                      const LieGroupBase<_DerivedOther>& state_other)
{
  state = state_other;
}

template <typename _Derived, typename _Other>
void move_assign(LieGroupBase<_Derived>& state,
                 _Other& state_other)
{
  state = std::move(state_other);
}

template <typename _Derived, typename _DerivedOther>
void move_assign_base(LieGroupBase<_Derived>& state,
                      LieGroupBase<_DerivedOther>& state_other)
{
  state = std::move(state_other);
}

} // namespace manif

#endif // _MANIF_MANIF_TEST_FUNCTIONS_H_
