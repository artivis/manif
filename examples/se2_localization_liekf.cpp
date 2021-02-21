/**
 * \file se2_localization_liekf.cpp
 *
 *  Created on: Jan 20, 2021
 *     \author: artivis
 *
 *  ---------------------------------------------------------
 *  This file is:
 *  (c) 2021 artivis
 *
 *  This file is part of `manif`, a C++ template-only library
 *  for Lie theory targeted at estimation for robotics.
 *  Manif is:
 *  (c) 2021 artivis
 *  ---------------------------------------------------------
 *
 *  ---------------------------------------------------------
 *  Demonstration example:
 *
 *  2D Robot localization based on position measurements (GPS-like)
 *  using the (Left) Invariant Extended Kalman Filter method.
 *
 *  See se2_localization_riekf.cpp for the right invariant equivalent
 *  ---------------------------------------------------------
 *
 *  This demo corresponds to the application in chapter 4.3 (left invariant)
 *  in the thesis
 *  'Non-linear state error based extended Kalman filters with applications to navigation' A. Barrau.
 *
 *  It is adapted after the sample problem presented in chapter 4.1 (left invariant)
 *  in the thesis
 *  'Practical Considerations and Extensions of the Invariant Extended Kalman Filtering Framework' J. Arsenault
 *
 *  Finally, it is ported from a Matlab example by Faraaz Ahmed, McGill University.
 *
 *  The following is an abstract of the content of the paper.
 *  Please consult the paper for better reference.
 *
 *
 *  We consider a robot in the plane.
 *  The robot receives control actions in the form of axial
 *  and angular velocities, and is able to measure its position
 *  using a GPS for instance.
 *
 *  The robot pose X is in SE(2) and the beacon positions b_k in R^2,
 *
 *          | cos th  -sin th   x |
 *      X = | sin th   cos th   y |  // position and orientation
 *          |   0        0      1 |
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
 *  GPS measurements are put in Cartesian form for simplicity.
 *  Their noise n is zero mean Gaussian, and is specified
 *  with a covariances matrix R.
 *  We notice the rigid motion action y = h(X) = X * [0 0 1]^T + v
 *
 *      y_k = (x, y)       // robot coordinates
 *
 *  We define the pose to estimate as X in SE(2).
 *  The estimation error dx and its covariance P are expressed
 *  in the global space at epsilon.
 *
 *  All these variables are summarized again as follows
 *
 *    X   : robot pose, SE(2)
 *    u   : robot control, (v*dt ; 0 ; w*dt) in se(2)
 *    Q   : control perturbation covariance
 *    y   : robot position measurement in global frame, R^2
 *    R   : covariance of the measurement noise
 *
 *  The motion and measurement models are
 *
 *    X_(t+1) = f(X_t, u) = X_t * Exp ( w )     // motion equation
 *    y_k     = h(X, b_k) = X * [0 0 1]^T       // measurement equation
 *
 *  The algorithm below comprises first a simulator to
 *  produce measurements, then uses these measurements
 *  to estimate the state, using a Lie-based
 *  Left Invariant Kalman filter.
 *
 *  This file has plain code with only one main() function.
 *  There are no function calls other than those involving `manif`.
 *
 *  Printing simulated state and estimated state together
 *  with an unfiltered state (i.e. without Kalman corrections)
 *  allows for evaluating the quality of the estimates.
 */

#include "manif/SE2.h"

#include <vector>

#include <iostream>
#include <iomanip>
#include <random>
#include <chrono>

#include <Eigen/Dense>

using std::cout;
using std::endl;

using namespace Eigen;

typedef Array<double, 2, 1> Array2d;
typedef Array<double, 3, 1> Array3d;

int main()
{
    // std::srand((unsigned int) time(0));

    std::default_random_engine generator{
        static_cast<long unsigned int>(std::chrono::high_resolution_clock::now().time_since_epoch().count())
    };
    std::normal_distribution<> distribution{0,1};
    // Random double in N(0, 1)
    auto randn = [&] (int) {return distribution(generator);};

    // START CONFIGURATION
    //
    //

    constexpr double dt = 0.01;     // s
    constexpr int gps_freq = 10;    // Hz

    double sqrtdt = std::sqrt(dt);

    constexpr double var_gyro = 1e-6;           // (rad/s)^2
    constexpr double var_wheel_odometry = 9e-6; // (m/s)^2
    constexpr double var_gps = 6e-3;            // m^2

    // Define the robot pose element and its covariance
    manif::SE2d X, X_simulation, X_unfiltered;
    manif::SE2d X_hat, X_hat_prev, X_check;
    Matrix3d    P, I;

    X_simulation.setIdentity();
    X.setIdentity();
    X_hat.setIdentity();
    X_hat_prev.setIdentity();
    X_check.setIdentity();
    X_unfiltered.setIdentity();
    P = Matrix3d::Identity();
    P(2,2) = MANIF_PI/6.;
    I.setIdentity();

    // Define a control vector and its noise and covariance
    manif::SE2Tangentd  u_simu, u_est, u_unfilt;
    Vector3d            u_nom, u_nom_simu, u_noisy, u_noise;
    Array3d             u_sigmas;
    Matrix3d            U;

    u_sigmas << std::sqrt(var_wheel_odometry), std::sqrt(var_wheel_odometry), std::sqrt(var_gyro);
    U        = (u_sigmas * u_sigmas * 1./dt).matrix().asDiagonal();

    // Declare the Jacobians of the motion wrt robot and control
    manif::SE2d::Jacobian F, G;
    G = -manif::SE2d::Jacobian::Identity() * dt;

    // Define the beacon's measurements
    Vector2d                y, y_noise;
    Array2d                 y_sigmas;
    Matrix2d                R;

    y_sigmas << std::sqrt(var_gps), std::sqrt(var_gps);
    R        = (y_sigmas * y_sigmas * 1./dt).matrix().asDiagonal();

    // Declare the Jacobian of the measurements wrt the robot pose
    Matrix<double, 2, 3> H;
    H.topRightCorner<2, 1>().setZero();
    H.topLeftCorner<2, 2>() = -Matrix2d::Identity();

    Matrix<double, 2, 2> M;

    // Declare some temporaries
    Vector2d                e, z;   // expectation, innovation
    Matrix2d                E, Z;   // covariances of the above
    Matrix<double, 3, 2>    K;      // Kalman gain
    manif::SE2Tangentd      dx;     // optimal update step, or error-state

    //
    //
    // CONFIGURATION DONE



    // DEBUG
    cout << std::fixed   << std::setprecision(6)   << std::showpos << "\n";
    cout << "X STATE     :    X      Y    THETA"   << "\n";
    cout << "----------------------------------"   << "\n";
    cout << "X initial   : " << X_simulation.log() << "\n";
    cout << "----------------------------------"   << endl;
    // END DEBUG




    // START TEMPORAL LOOP
    //
    //

    for (double t = 0; t < 30; t += dt)
    {
        //// I. Simulation ###############################################################################

        if (t < dt) {
            u_nom << 0, 0, 0;
            u_nom_simu << 0.1 * std::cos(dt) + 10.0,
                     0.0,
                     std::exp(-0.03 * (dt)) * std::cos(dt);
        }
        else {
            u_nom << 0.1 * std::cos(t-dt) + 10.0,
                    0.0,
                    std::exp(-0.03 * (t-dt)) * std::cos(t-dt);

            u_nom_simu << 0.1 * std::cos(t) + 10.0,
                        0.0,
                        std::exp(-0.03 * (t)) * std::cos(t);

        }

        /// simulate noise
        u_noise = u_sigmas / sqrtdt * Array3d::Zero().unaryExpr(randn);     // control noise
        u_noisy = u_nom + u_noise;                                          // noisy control

        u_simu   = u_nom_simu * dt;
        u_est    = u_noisy * dt;
        u_unfilt = u_noisy * dt;

        /// first we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        X_simulation = X_simulation + u_simu;                   // overloaded X.rplus(u) = X * exp(u)


        //// II. Estimation ###############################################################################

        /// First we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        X_check = X_hat_prev.plus(u_est);                                      // X * exp(u)

        F = (-u_est).exp().adj();

        P = F * P * F.transpose() + G * U * G.transpose();

        /// Then we correct using the measurements - - - - - - - - -
        if (int(t*100) % gps_freq == 0) { // ~ update once every 10 dt
            /// Generate noisy measurement
            y_noise = y_sigmas / sqrtdt * Array2d::Zero().unaryExpr(randn);        // measurement noise

            y = X_simulation.translation();                     // position measurement, before adding noise
            y = y + y_noise;                                    // position measurement, noisy

            // expectation
            e = X_check.translation();                          // note: e = t, for X = (R,t).

            M = X_check.rotation().transpose();                 // note: M = R^T, for X = (R,t).

            E = H * P * H.transpose();

            // innovation
            z = X_check.inverse().rotation() * (y - e);         // z = X^-1 * vec
            Z = E + M * R * M.transpose();

            // Kalman gain
            K = P * H.transpose() * Z.inverse();                // K = P * H.tr * ( H * P * H.tr + R).inv

            // Correction step
            dx = K * z;                                         // dx is in the tangent space

            // Update
            X_hat = X_check + (-dx);                            // X * exp(-dx)

            P = (I - K * H) * P * (I - K * H).transpose() +
                K * M * R * M.transpose() * K.transpose();
        }
        else
        {
            X_hat = X_check;
        }

        X_hat_prev = X_hat;


        //// III. Unfiltered ##############################################################################

        // move also an unfiltered version for comparison purposes
        X_unfiltered = X_unfiltered + u_unfilt;


        //// IV. Results ##############################################################################

        // DEBUG
        cout << "X simulated : " << X_simulation.x() << " " << X_simulation.y() << " " << X_simulation.angle() << "\n";
        cout << "X estimated : " << X_hat.x() << " " << X_hat.y() << " " << X_hat.angle()
                                 << ". |d|: " << (X_simulation - X_hat).squaredWeightedNorm() << "\n";
        cout << "X unfilterd : " << X_unfiltered.x() << " " << X_unfiltered.y() << " " << X_unfiltered.angle()
                                 << ". |d|: " << (X_simulation - X_unfiltered).squaredWeightedNorm() << "\n";
        cout << "----------------------------------" << endl;
        // END DEBUG

    }

    //
    //
    // END OF TEMPORAL LOOP. DONE.

    return 0;
}
