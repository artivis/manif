/**
 * \file se2_localization_iekf.cpp
 *
 *  Created on: Feb 13, 2018
 *     \author: artivis
 *
 *  ---------------------------------------------------------
 *  This file is:
 *  (c) 2021 Jeremie Deray
 *
 *  This file is part of `manif`, a C++ template-only library
 *  for Lie theory targeted at estimation for robotics.
 *  Manif is:
 *  (c) 2018 Jeremie Deray
 *  ---------------------------------------------------------
 *
 *  ---------------------------------------------------------
 *  Demonstration example:
 *
 *  2D Robot localization based on fixed beacons using the
 *  (Right) Invariant Extended Kalman Filter method.
 *
 *  See se2_localization_liekf.cpp for the left invariant equivalent
 *  ---------------------------------------------------------
 *
 *  This demo corresponds to the application in chapter 4.3 (right invariant)
 *  in the thesis
 *  'Non-linear state error based extended Kalman filters with applications to navigation' A. Barrau.
 *
 *  It is adapted after the sample problem presented in chapter 4.1 (right invariant)
 *  in the thesis
 *  'Practical Considerations and Extensions of the Invariant Extended Kalman Filtering Framework' J. Arsenault
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
 *  to estimate the state, using a Lie-based error-state Invariant Kalman filter.
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

using std::cout;
using std::endl;

using namespace Eigen;

typedef Array<double, 2, 1> Array2d;
typedef Array<double, 3, 1> Array3d;

int main()
{
    std::srand((unsigned int) time(0));

    // START CONFIGURATION
    //
    //
    const int NUMBER_OF_LMKS_TO_MEASURE = 3;

    const Matrix3d I = Matrix3d::Identity();

    // Define the robot pose element and its covariance
    manif::SE2d X, X_simulation, X_unfiltered;
    Matrix3d    P, P_L;

    X_simulation.setIdentity();
    X.setIdentity();
    X_unfiltered.setIdentity();
    P = Matrix3d::Identity() * 1e-5;

    // Define a control vector and its noise and covariance
    manif::SE2Tangentd  u_simu, u_est, u_unfilt;
    Vector3d            u_nom, u_noisy, u_noise;
    Array3d             u_sigmas;
    Matrix3d            U;

    u_nom    << 0.1, 0.0, 0.05;
    u_sigmas << 0.1, 0.1, 0.025;
    U        = (u_sigmas * u_sigmas).matrix().asDiagonal();

    // Declare the Jacobians of the motion wrt robot and control
    manif::SE2d::Jacobian F, G, AdX, AdXinv;
    F.setIdentity();

    // Define three landmarks in R^2
    Vector2d b;
    const std::vector<Vector2d> landmarks{
      Vector2d(2.0,  0.0),
      Vector2d(2.0,  1.0),
      Vector2d(2.0, -1.0)
    };

    // Define the beacon's measurements
    Vector2d                y, y_noise;
    Array2d                 y_sigmas;
    Matrix2d                R;
    std::vector<Vector2d>   measurements(landmarks.size());

    y_sigmas << 0.01, 0.01;
    R        = (y_sigmas * y_sigmas).matrix().asDiagonal();

    // Declare the Jacobian of the measurements wrt the robot pose
    Matrix<double, 2, 3>    H;
    H.topLeftCorner<2, 2>().setIdentity();

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
    cout << std::fixed   << std::setprecision(3) << std::showpos << "\n"
         << "X STATE     :    X      Y    THETA" << "\n"
         << "----------------------------------" << "\n"
         << "X initial   : " << X_simulation.log() << "\n"
         << "----------------------------------" << endl;
    // END DEBUG




    // START TEMPORAL LOOP
    //
    //

    // Make 10 steps. Measure up to three landmarks each time.
    for (int t = 0; t < 30; t++)
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

        X = X + u_est;                        // X * exp(u)

        G = -(X.adj());

        P = F * P * F.transpose() + G * U * G.transpose();


        /// Then we correct using the measurements of each lmk - - - - - - - - -
        for (int i = 0; i < NUMBER_OF_LMKS_TO_MEASURE; i++)
        {
            // landmark
            b = landmarks[i];                               // lmk coordinates in world frame

            // measurement
            y = measurements[i];                            // lmk measurement, noisy

            // expectation
            e = X.inverse().act(b);                         // note: e = R.tr * ( b - t ), for X = (R,t).

            M = X.rotation();

            H.topLeftCorner<2, 2>().setIdentity();
            H.topRightCorner<2, 1>() = manif::skew(double(1)) * b;

            E = H * P * H.transpose();

            // innovation
            z = X.rotation() * (y - e);
            Z = E + M * R * M.transpose();

            // Kalman gain
            K = P * H.transpose() * Z.inverse();            // K = P * H.tr * ( H * P * H.tr + R).inv

            // Correction step
            dx = K * z;                                     // dx is in the tangent space

            // Update
            X = (-dx) + X;                                  // exp(-dx) * X

            P = (I - K * H) * P;
        }

        // GPS measurement update
        // Update every 3 dt
        if (t % 3 == 0) {
            // measurement
            y_noise = y_sigmas * Array2d::Random();         // measurement noise

            y = X_simulation.translation();                 // position measurement, before adding noise
            y = y + y_noise;                                // position measurement, noisy

            // expectation
            e = X.translation();                            // note: e = t, for X = (R,t).

            M = X.rotation().transpose();                   // note: M = R^T, for X = (R,t).

            AdX = X.adj();
            AdXinv = X.inverse().adj();

            P_L = AdXinv * P * AdXinv.transpose();          // note: transform covariance from right to left invariant

            H.topRightCorner<2, 1>().setZero();
            H.topLeftCorner<2, 2>() = -Matrix2d::Identity();

            E = H * P_L * H.transpose();

            // innovation
            z = X.inverse().rotation() * (y - e);

            Z = E + M * R * M.transpose();

            // Kalman gain
            K = P_L * H.transpose() * Z.inverse();              // K = P * H.tr * ( H * P * H.tr + R).inv

            // Correction step
            dx = K * z;                                         // dx is in the tangent space

            // Update
            X = X + (-dx);                                      // exp(-dx) * X

            P = AdX * ((I - K * H) * P_L) * AdX.transpose();    // note: transform covariance from left to right invariant
        }



        //// III. Unfiltered ##############################################################################

        // move also an unfiltered version for comparison purposes
        X_unfiltered = X_unfiltered + u_unfilt;




        //// IV. Results ##############################################################################

        // DEBUG
        cout << "X simulated : " << X_simulation.log() << "\n";
        cout << "X estimated : " << X.log() << " |d| "
             << (X_simulation - X).squaredWeightedNorm() << "\n";
        cout << "X unfilterd : " << X_unfiltered.log() << " |d| "
             << (X_simulation - X_unfiltered).squaredWeightedNorm() << "\n";
        cout << "----------------------------------" << endl;
        // END DEBUG

    }

    //
    //
    // END OF TEMPORAL LOOP. DONE.

    return 0;
}
