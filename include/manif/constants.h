#ifndef _MANIF_CONSTANTS_H_
#define _MANIF_CONSTANTS_H_

namespace manif
{

//template <typename _Scalar>
//struct Constants
//{
//  static constexpr _Scalar eps      = _Scalar(1e-10);
//  static constexpr _Scalar eps_sq   = eps*eps;
//  static constexpr _Scalar eps_sqrt = sqrt(eps);

//  static constexpr _Scalar to_rad =
//      _Scalar(0.017453292519943295769236907684886127134); // pi / 180
//  static constexpr _Scalar to_deg =
//      _Scalar(57.295779513082320876798154814105170332); // 180 / pi
//};

template <typename _Scalar>
struct Constants
{
  static const _Scalar eps     ;
  static const _Scalar eps_sq  ;
  static const _Scalar eps_sqrt;

  static const _Scalar to_rad; // pi / 180
  static const _Scalar to_deg; // 180 / pi
};

template <typename _Scalar>
const _Scalar Constants<_Scalar>::eps = _Scalar(1e-10);

template <typename _Scalar>
const _Scalar Constants<_Scalar>::eps_sq = _Scalar(1e-10*1e-10);

template <typename _Scalar>
const _Scalar Constants<_Scalar>::eps_sqrt = _Scalar(std::sqrt(1e-10));

template <typename _Scalar>
const _Scalar Constants<_Scalar>::to_rad = _Scalar(0.017453292519943295769236907684886127134);

template <typename _Scalar>
const _Scalar Constants<_Scalar>::to_deg = _Scalar(57.295779513082320876798154814105170332);

} /* namespace manif  */

#endif /* _MANIF_CONSTANTS_H_ */
