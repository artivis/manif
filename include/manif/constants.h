#ifndef _MANIF_CONSTANTS_H_
#define _MANIF_CONSTANTS_H_

namespace manif
{

template <typename _Scalar>
struct Constants
{
  static constexpr _Scalar eps      = _Scalar(1e-10);
  static constexpr _Scalar eps_sq   = eps*eps;
  static constexpr _Scalar eps_sqrt = sqrt(eps);
};

} /* namespace manif  */

#endif /* _MANIF_CONSTANTS_H_ */
