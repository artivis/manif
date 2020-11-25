/**
 * \file se2_localization.cpp
 *
 *  Created on: Dec 10, 2018
 *     \author: jsola
 *
 *  ---------------------------------------------------------
 *  This file is:
 *  (c) 2018 Joan Sola @ IRI-CSIC, Barcelona, Catalonia
 *
 *  This file is part of `manif`, a C++ template-only library
 *  for Lie theory targeted at estimation for robotics.
 *  Manif is:
 *  (c) 2018 Jeremie Deray @ IRI-UPC, Barcelona
 *  ---------------------------------------------------------
 *
 *  ---------------------------------------------------------
 *  Demonstration example:
 *
 *  2D Robot localization based on fixed beacons.
 *
 *  See se3_localization.cpp for the 3D equivalent.
 *  See se3_sam.cpp for a more advanced example performing smoothing and mapping.
 *  ---------------------------------------------------------
 *
 *  This demo showcases an application of an Unscented Kalman Filter on Manifold,
 *  based on the paper
 *  'A Code for Unscented Kalman Filtering on Manifolds (UKF-M)'
 *  [https://arxiv.org/pdf/2002.00878.pdf].
 *
 *  The following is an abstract of the example hereafter.
 *  Please consult the aforemention paper for better UKF-M reference
 *  and the paper Sola-18, [https://arxiv.org/abs/1812.01537] for general
 *  Lie group reference.
 *
 *
 *  We consider a robot in the plane surrounded by a small
 *  number of punctual landmarks or _beacons_.
 *  The robot receives control actions in the form of axial
 *  and angular velocities, and is able to measure the location
 *  of the beacons w.r.t its own reference frame.
 *
 *  The robot pose X is in SE(2) and the beacon positions b_k in R^2,
 *
 *          | cos th  -sin th   x |
 *      X = | sin th   cos th   y |  // position and orientation
 *          |   0        0      1 |
 *
 *      b_k = (bx_k, by_k)           // lmk coordinates in world frame
 *
 *  The control signal u is a twist in se(2) comprising longitudinal
 *  velocity v and angular velocity w, with no lateral velocity
 *  component, integrated over the sampling time dt.
 *
 *      u = (v*dt, 0, w*dt)
 *
 *  The control is corrupted by additive Gaussian noise u_noise,
 *  with covariance
 *
 *    Q = diagonal(sigma_v^2, sigma_s^2, sigma_w^2).
 *
 *  This noise accounts for possible lateral slippage u_s
 *  through a non-zero value of sigma_s,
 *
 *  At the arrival of a control u, the robot pose is updated
 *  with X <-- X * Exp(u) = X + u.
 *
 *  Landmark measurements are of the range and bearing type,
 *  though they are put in Cartesian form for simplicity.
 *  Their noise n is zero mean Gaussian, and is specified
 *  with a covariances matrix R.
 *  We notice the rigid motion action y = h(X,b) = X^-1 * b
 *  (see appendix C),
 *
 *      y_k = (brx_k, bry_k)       // lmk coordinates in robot frame
 *
 *  We consider the beacons b_k situated at known positions.
 *  We define the pose to estimate as X in SE(2).
 *  The estimation error dx and its covariance P are expressed
 *  in the tangent space at X.
 *
 *  All these variables are summarized again as follows
 *
 *    X   : robot pose, SE(2)
 *    u   : robot control, (v*dt ; 0 ; w*dt) in se(2)
 *    Q   : control perturbation covariance
 *    b_k : k-th landmark position, R^2
 *    y   : Cartesian landmark measurement in robot frame, R^2
 *    R   : covariance of the measurement noise
 *
 *  The motion and measurement models are
 *
 *    X_(t+1) = f(X_t, u) = X_t * Exp ( w )     // motion equation
 *    y_k     = h(X, b_k) = X^-1 * b_k          // measurement equation
 *
 *  The algorithm below comprises first a simulator to
 *  produce measurements, then uses these measurements
 *  to estimate the state, using a Lie-based error-state Kalman filter.
 *
 *  This file has plain code with only one main() function.
 *  There are no function calls other than those involving `manif`.
 *
 *  Printing simulated state and estimated state together
 *  with an unfiltered state (i.e. without Kalman corrections)
 *  allows for evaluating the quality of the estimates.
 */

#include "manif/SE2.h"

#include <Eigen/Cholesky>

#include <vector>

#include <iostream>
#include <iomanip>
#include <tuple>

using std::cout;
using std::endl;

using namespace Eigen;

typedef Array<double, 2, 1> Array2d;
typedef Array<double, 3, 1> Array3d;

template <typename Scalar>
struct Weights
{
  Weights() = default;
  ~Weights() = default;

  Weights(const Scalar l, const Scalar alpha)
  {
    using std::sqrt;

    const Scalar m = (alpha * alpha - 1) * l;
    const Scalar ml = m + l;

    sqrt_d_lambda = sqrt(ml);
    wj = Scalar(1) / (Scalar(2) * (ml));
    wm = m / (ml);
    w0 = m / (ml) + Scalar(3) - alpha * alpha;
  }

  Scalar sqrt_d_lambda;
  Scalar wj;
  Scalar wm;
  Scalar w0;
};

using Weightsd = Weights<double>;

template <typename Scalar>
std::tuple<Weights<Scalar>, Weights<Scalar>, Weights<Scalar>>
compute_sigma_weights(const Scalar state_size,
                      const Scalar propagation_noise_size,
                      const Scalar alpha_0,
                      const Scalar alpha_1,
                      const Scalar alpha_2)
{
  assert(state_size>0);
  assert(propagation_noise_size>0);
  assert(alpha_0>=1e-3 && alpha_0<=1);
  assert(alpha_1>=1e-3 && alpha_1<=1);
  assert(alpha_2>=1e-3 && alpha_2<=1);

  return std::make_tuple(Weights<Scalar>(state_size, alpha_0),
                         Weights<Scalar>(propagation_noise_size, alpha_1),
                         Weights<Scalar>(state_size, alpha_2));
}

int main()
{
    // START CONFIGURATION
    //
    //
    const int NUMBER_OF_LMKS_TO_MEASURE = 3;
    constexpr int DoF = manif::SE2d::DoF;
    // Measurement Dim
    constexpr int Rp = 2;

    // Define the robot pose element and its covariance
    manif::SE2d X            = manif::SE2d::Identity(),
                X_simulation = manif::SE2d::Identity(),
                X_unfiltered = manif::SE2d::Identity();
    Matrix3d    P            = Matrix3d::Identity() * 1e-6;

    // Define a control vector and its noise and covariance
    manif::SE2Tangentd  u_simu, u_est, u_unfilt;
    Vector3d            u_nom, u_noisy, u_noise;
    Array3d             u_sigmas;
    Matrix3d            U, Uchol;

    u_nom    << 0.1, 0.0, 0.05;
    u_sigmas << 0.1, 0.1, 0.1;
    U        = (u_sigmas * u_sigmas).matrix().asDiagonal();
    Uchol    = U.llt().matrixL();

    // Define three landmarks in R^2
    Eigen::Vector2d b;
    const std::vector<Eigen::Vector2d> landmarks{
      Eigen::Vector2d(2.0,  0.0),
      Eigen::Vector2d(2.0,  1.0),
      Eigen::Vector2d(2.0, -1.0)
    };

    // Define the beacon's measurements
    Vector2d                  y, y_bar, y_noise;
    Matrix<double, Rp, 2*DoF> yj;
    Array2d                   y_sigmas;
    Matrix2d                  R;
    std::vector<Vector2d>     measurements(landmarks.size());

    y_sigmas << 0.01, 0.01;
    R        = (y_sigmas * y_sigmas).matrix().asDiagonal();


    // Declare UFK variables
    Array3d alpha;
    alpha << 1e-3, 1e-3, 1e-3;

    Weightsd w_d, w_q, w_u;
    std::tie(w_d, w_q, w_u) = compute_sigma_weights<double>(
      DoF, Rp, alpha(0), alpha(1), alpha(2)
    );

    // Declare some temporaries

    manif::SE2d X_new;
    Matrix3d P_new;
    manif::SE2d s_j_p, s_j_m;
    Vector3d xi_mean;
    Vector3d w_p, w_m;

    Matrix2d P_yy;
    Matrix<double, DoF, 2*DoF> xij;
    Matrix<double, DoF, 2> P_xiy;

    Vector2d                e, z;   // expectation, innovation
    Matrix<double, 3, 2>    K;      // Kalman gain
    manif::SE2Tangentd      dx;     // optimal update step, or error-state

    Matrix<double, DoF, DoF> xis;
    Matrix<double, DoF, DoF*2> xis_new;
    Matrix<double, DoF, 2*2> xis_new2;

    //
    //
    // CONFIGURATION DONE



    // DEBUG
    cout << std::fixed   << std::setprecision(4) << std::showpos << endl;
    cout << "X STATE     :    X      Y    THETA" << endl;
    cout << "----------------------------------" << endl;
    cout << "X initial   : " << X_simulation.log().coeffs().transpose() << endl;
    cout << "----------------------------------" << endl;
    // END DEBUG



    // START TEMPORAL LOOP
    //
    //

    // Make 10 steps. Measure up to three landmarks each time.
    for (int t = 0; t <10; t++)
    {
        //// I. Simulation ###############################################################################

        /// simulate noise
        u_noise = u_sigmas * Array3d::Random();             // control noise
        u_noisy = u_nom + u_noise;                          // noisy control

        u_simu   = u_nom;
        u_est    = u_noisy;
        u_unfilt = u_noisy;

        /// first we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        X_simulation = X_simulation + u_simu;               // overloaded X.rplus(u) = X * exp(u)

        /// then we measure all landmarks - - - - - - - - - - - - - - - - - - - -
        for (int i = 0; i < landmarks.size(); i++)
        {
            b = landmarks[i];                               // lmk coordinates in world frame

            /// simulate noise
            y_noise = y_sigmas * Array2d::Random();         // measurement noise

            y = X_simulation.inverse().act(b);              // landmark measurement, before adding noise
            y = y + y_noise;                                // landmark measurement, noisy
            measurements[i] = y;                            // store for the estimator just below
        }




        //// II. Estimation ###############################################################################

        /// First we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        X_new = X + u_est;                        // X * exp(u)

        // set sigma points
        Matrix3d PLt = P.llt().matrixL();
        xis = w_d.sqrt_d_lambda * PLt;

        // sigma points on manifold
        for (int i = 0; i < DoF; ++i)
        {
          s_j_p = X + manif::SE2Tangentd( xis.col(i));
          s_j_m = X + manif::SE2Tangentd(-xis.col(i));

          xis_new.col(i) = (X_new - (s_j_p + u_est)).coeffs();
          xis_new.col(i + DoF) = (X_new - (s_j_m + u_est)).coeffs();
        }

        // compute covariance
        xi_mean = w_d.wj * xis_new.rowwise().sum();
        xis_new.colwise() -= xi_mean;

        P_new = w_d.wj * xis_new * xis_new.transpose() +
                w_d.w0 * xi_mean * xi_mean.transpose();

        // sigma points on manifold
        for (int i = 0; i < 2; ++i)
        {
          w_p =  w_q.sqrt_d_lambda * Uchol.col(i);
          w_m = -w_q.sqrt_d_lambda * Uchol.col(i);

          xis_new2.col(i) = (X_new - (X + (u_est + w_p))).coeffs();
          xis_new2.col(i + 2) = (X_new - (X + (u_est + w_m))).coeffs();
        }

        xi_mean = w_q.wj * xis_new2.rowwise().sum();
        xis_new2.colwise() -= xi_mean;

        U = w_q.wj * xis_new2 * xis_new2.transpose() +
            w_q.w0 * xi_mean * xi_mean.transpose();

        P = P_new + U;

        X = X_new;

        /// Then we correct using the measurements of each lmk - - - - - - - - -
        for (int i = 0; i < NUMBER_OF_LMKS_TO_MEASURE; i++)
        {
            // landmark
            b = landmarks[i];                               // lmk coordinates in world frame

            // measurement
            y = measurements[i];                            // lmk measurement, noisy

            // expectation
            e = X.inverse().act(b);

            // set sigma points
            PLt = P.llt().matrixL();
            xis = w_d.sqrt_d_lambda * PLt;

            // compute measurement sigma points
            for (int i = 0; i < DoF; ++i)
            {
              s_j_p = X + manif::SE2Tangentd( xis.col(i));
              s_j_m = X + manif::SE2Tangentd(-xis.col(i));

              yj.col(i) = s_j_p.inverse().act(b);
              yj.col(i + DoF) = s_j_m.inverse().act(b);
            }

            // measurement mean
            y_bar = w_d.wm * e + w_d.wj * yj.rowwise().sum();

            yj.colwise() -= y_bar;
            e -= y_bar;

            // compute covariance and cross covariance matrices
            P_yy = w_u.w0 * e * e.transpose()   +
                   w_u.wj * yj * yj.transpose() + R;

            xij << xis, -xis;
            P_xiy = w_u.wj * xij * yj.transpose();

            // Kalman gain
            K = P_yy.colPivHouseholderQr().solve(P_xiy.transpose()).transpose();

            // innovation
            z = y - y_bar;

            // Correction step
            dx = K * z;                                     // dx is in the tangent space at X

            // Update
            X = X + dx;                                     // overloaded X.rplus(dx) = X * exp(dx)
            P = P - K * P_yy * K.transpose();
        }


        //// III. Unfiltered ##############################################################################

        // move also an unfiltered version for comparison purposes
        X_unfiltered = X_unfiltered + u_unfilt;




        //// IV. Results ##############################################################################

        // DEBUG
        cout << "X simulated : " << X_simulation.log().coeffs().transpose() << "." << endl;
        cout << "X estimated : " << X.log().coeffs().transpose()
             << ". |d|=" << (X_simulation - X).squaredWeightedNorm() <<endl;
        cout << "X unfilterd : " << X_unfiltered.log().coeffs().transpose()
             << ". |d|=" << (X_simulation - X_unfiltered).squaredWeightedNorm() <<endl;
        cout << "----------------------------------" << endl;
        // END DEBUG

    }

    //
    //
    // END OF TEMPORAL LOOP. DONE.

    return 0;
}
