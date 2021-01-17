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
 *  This demo corresponds to the application in chapter V, section A,
 *  in the paper Sola-18, [https://arxiv.org/abs/1812.01537].
 *
 *  The following is an abstract of the content of the paper.
 *  Please consult the paper for better reference.
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

    // Define the robot pose element and its covariance
    manif::SE2d X, X_simulation, X_unfiltered;
    Matrix3d    P;

    X_simulation.setIdentity();
    X.setIdentity();
    X_unfiltered.setIdentity();
    P.setZero();

    // Define a control vector and its noise and covariance
    manif::SE2Tangentd  u_simu, u_est, u_unfilt;
    Vector3d            u_nom, u_noisy, u_noise;
    Array3d             u_sigmas;
    Matrix3d            U;

    u_nom    << 0.1, 0.0, 0.05;
    u_sigmas << 0.1, 0.1, 0.1;
    U        = (u_sigmas * u_sigmas).matrix().asDiagonal();

    // Declare the Jacobians of the motion wrt robot and control
    manif::SE2d::Jacobian J_x, J_u;

    // Define three landmarks in R^2
    Eigen::Vector2d b0, b1, b2, b;
    b0 << 2.0,  0.0;
    b1 << 2.0,  1.0;
    b2 << 2.0, -1.0;
    std::vector<Eigen::Vector2d> landmarks;
    landmarks.push_back(b0);
    landmarks.push_back(b1);
    landmarks.push_back(b2);

    // Define the beacon's measurements
    Vector2d                y, y_noise;
    Array2d                 y_sigmas;
    Matrix2d                R;
    std::vector<Vector2d>   measurements(landmarks.size());

    y_sigmas << 0.01, 0.01;
    R        = (y_sigmas * y_sigmas).matrix().asDiagonal();

    // Declare the Jacobian of the measurements wrt the robot pose
    Matrix<double, 2, 3>    H;      // H = J_e_x

    // Declare some temporaries
    Vector2d                e, z;   // expectation, innovation
    Matrix2d                E, Z;   // covariances of the above
    Matrix<double, 3, 2>    K;      // Kalman gain
    manif::SE2Tangentd      dx;     // optimal update step, or error-state
    manif::SE2d::Jacobian   J_xi_x; // Jacobian is typedef Matrix
    Matrix<double, 2, 3>    J_e_xi; // Jacobian

    //
    //
    // CONFIGURATION DONE



    // DEBUG
    cout << std::fixed   << std::setprecision(3) << std::showpos << endl;
    cout << "X STATE     :    X      Y    THETA" << endl;
    cout << "----------------------------------" << endl;
    cout << "X initial   : " << X_simulation.log().coeffs().transpose() << endl;
    cout << "----------------------------------" << endl;
    // END DEBUG




    // START TEMPORAL LOOP
    //
    //

    // Make 10 steps. Measure up to three landmarks each time.
    for (int t = 0; t < 10; t++)
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

        X = X.plus(u_est, J_x, J_u);                        // X * exp(u), with Jacobians

        P = J_x * P * J_x.transpose() + J_u * U * J_u.transpose();


        /// Then we correct using the measurements of each lmk - - - - - - - - -
        for (int i = 0; i < NUMBER_OF_LMKS_TO_MEASURE; i++)
        {
            // landmark
            b = landmarks[i];                               // lmk coordinates in world frame

           // measurement
            y = measurements[i];                            // lmk measurement, noisy

            // expectation
            e = X.inverse(J_xi_x).act(b, J_e_xi);           // note: e = R.tr * ( b - t ), for X = (R,t).
            H = J_e_xi * J_xi_x;                            // note: H = J_e_x = J_e_xi * J_xi_x
            E = H * P * H.transpose();

            // innovation
            z = y - e;
            Z = E + R;

            // Kalman gain
            K = P * H.transpose() * Z.inverse();            // K = P * H.tr * ( H * P * H.tr + R).inv

            // Correction step
            dx = K * z;                                     // dx is in the tangent space at X

            // Update
            X = X + dx;                                     // overloaded X.rplus(dx) = X * exp(dx)
            P = P - K * Z * K.transpose();
        }




        //// III. Unfiltered ##############################################################################

        // move also an unfiltered version for comparison purposes
        X_unfiltered = X_unfiltered + u_unfilt;




        //// IV. Results ##############################################################################

        // DEBUG
        cout << "X simulated : " << X_simulation.log().coeffs().transpose() << endl;
        cout << "X estimated : " << X.log().coeffs().transpose() << endl;
        cout << "X unfilterd : " << X_unfiltered.log().coeffs().transpose() << endl;
        cout << "----------------------------------" << endl;
        // END DEBUG

    }

    //
    //
    // END OF TEMPORAL LOOP. DONE.

    return 0;
}
