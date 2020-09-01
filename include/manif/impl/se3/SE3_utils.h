#ifndef _MANIF_MANIF_SE3_UTILS_H_
#define _MANIF_MANIF_SE3_UTILS_H_

namespace manif {
namespace internal {

template <typename EigenDerived>
void fillQ(Eigen::Ref<Eigen::Matrix<typename EigenDerived::Scalar, 3, 3>> Q,
           const Eigen::MatrixBase<EigenDerived>& c)
{
  using Scalar = typename EigenDerived::Scalar;
  static_assert_dim(Q, 3, 3);

  using std::cos;
  using std::sin;
  using std::sqrt;

  const Scalar theta_sq = c.template tail<3>().squaredNorm();

  Scalar A(0.5), B, C, D;

  // Small angle approximation
  if (theta_sq <= Constants<Scalar>::eps_s)
  {
    B =  Scalar(1./6.)  + Scalar(1./120.)  * theta_sq;
    C = -Scalar(1./24.) + Scalar(1./720.)  * theta_sq;
    D = -Scalar(1./60.);
  }
  else
  {
    const Scalar theta     = sqrt(theta_sq);
    const Scalar sin_theta = sin(theta);
    const Scalar cos_theta = cos(theta);
    B = (theta - sin_theta) / (theta_sq*theta);
    C = (Scalar(1) - theta_sq/Scalar(2) - cos_theta) / (theta_sq*theta_sq);
    D = (C - Scalar(3)*(theta-sin_theta-theta_sq*theta/Scalar(6)) / (theta_sq*theta_sq*theta));
    // http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17_identities.pdf
//    C = (theta_sq+Scalar(2)*cos_theta-Scalar(2)) / (Scalar(2)*theta_sq*theta_sq);
//    D = (Scalar(2)*theta - Scalar(3)*sin_theta + theta*cos_theta) / (Scalar(2)*theta_sq*theta_sq*theta);
  }

  /// @note Barfoot14tro Eq. 102
  const Eigen::Matrix<Scalar, 3, 3> V   = skew(c.template head<3>());
  const Eigen::Matrix<Scalar, 3, 3> W   = skew(c.template tail<3>());
  const Eigen::Matrix<Scalar, 3, 3> VW  = V * W;
  const Eigen::Matrix<Scalar, 3, 3> WV  = VW.transpose();       // Note on this change wrt. Barfoot: it happens that V*W = (W*V).transpose() !!!
  const Eigen::Matrix<Scalar, 3, 3> WVW = WV * W;
  const Eigen::Matrix<Scalar, 3, 3> VWW = VW * W;

  Q.noalias() =
      + A * V
      + B * (WV + VW + WVW)
      - C * (VWW - VWW.transpose() - Scalar(3) * WVW)           // Note on this change wrt. Barfoot: it happens that V*W*W = -(W*W*V).transpose() !!!
      - D * WVW * W;                                            // Note on this change wrt. Barfoot: it happens that W*V*W*W = W*W*V*W !!!
}

} // namespace manif
} // namespace internal

#endif // _MANIF_MANIF_SE3_UTILS_H_
