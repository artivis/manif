/**
 * \file se2_localization.cpp
 *
 *  Created on: Dec 10, 2018
 *     \author: jsola
 *
 *
 *  Robot localization based on observation of fixed beacons.
 *  ---------------------------------------------------------
 *
 *  This demo corresponds to the application in chapter V, section A in the paper Sola-18, [https://arxiv.org/abs/1812.01537].
 *  The following is an abstract of the content of the paper.
 *  Please consult the paper for better reference.
 *
 *  We consider a robot in the plane surrounded by a small number of punctual landmarks or \emph{beacons}.
 *  The robot receives control actions in the form of axial and angular velocities,
 *  and is able to measure the location of the beacons \wrt its own reference frame.
 *
 *  The robot pose is in $\SE(2)$ and the beacon positions in $\bbR^2$,
 *
 *  The control signal $\bfu$ is a twist in $\se(2)$ comprising longitudinal velocity $v$
 *  and angular velocity $\omega$, with no lateral velocity component, integrated over the sampling time $\dt$.
 *
 *  The control is corrupted by additive Gaussian noise $\bfw\sim\cN(\bf0,\bfQ)$,
 *  with covariance $Q=diagonal(\sigma_v^2, \sigma_s^2, \sigma_\omega^2)$.
 *  This noise accounts for possible lateral slippages $u_s$ through a value of $\sigma_s\ne0$,
 *
 *  At the arrival of a control $\bfu$, the robot pose is updated with X <-- X * Exp(u) = X + u.
 *
 *  Landmark measurements are of the range and bearing type, though they are put in Cartesian form for simplicity.
 *  Their noise $\bfn\sim\cN({\bf0},\bfR)$ is zero mean Gaussian,% and is specified with a covariances matrix $R$.
 *  We notice the rigid motion action $\cX\inv\cdot\bfb_k$ (see appendix C).
 *
 *  We consider the beacons $\bfb_k$ situated at known positions.
 *  We define the pose to estimate as $\hat\cX\in\SE(2)$.
 *  The estimation error $\dx$ and its covariance $\bfP$ are expressed in the tangent space at $\hat\cX$.
 *
 */

#include "manif/SE2.h"

#include <Eigen/Dense>

#include <vector>

#include <iostream>
#include <iomanip>

using std::cout;
using std::endl;

int main()
{
    // START CONFIGURATION
    //
    //
    const int NUMBER_OF_LMKS_TO_MEASURE = 3;

    // Define the robot pose element and its covariance
    manif::SE2d X, X_simulation;
    Eigen::Matrix3d P;

    X_simulation.setIdentity();
    X.setIdentity();
    P.setZero();

    // Define a control vector and its noise and covariance
    manif::SE2Tangentd u_simu, u_est;
    Eigen::Vector3d u, u_noisy, u_sigmas, u_noise;
    Eigen::Matrix3d U;

    u = (Eigen::Vector3d() << 0.1, 0.0, 0.05).finished();
    u_sigmas << 0.1, 0.1, 0.1;
    U = (u_sigmas.array() * u_sigmas.array()).matrix().asDiagonal();

    // Declare the Jacobians of the motion wrt robot and control
    manif::SE2d::Jacobian J_x, J_u;

    // Define three landmarks in R^2
    Eigen::Vector2d b0, b1, b2, b;
    b0 << 2.0, 0.0;
    b1 << 2.0, 1.0;
    b2 << 2.0, -1.0;
    std::vector<Eigen::Vector2d> landmarks;
    landmarks.push_back(b0);
    landmarks.push_back(b1);
    landmarks.push_back(b2);

    // Define the beacon's measurements
    Eigen::Vector2d y;
    Eigen::Matrix2d R;
    Eigen::Vector2d n_sigmas, y_noise; 
    std::vector<Eigen::Vector2d> measurements(landmarks.size());

    n_sigmas << 0.01, 0.01;
    R = (n_sigmas.array() * n_sigmas.array()).matrix().asDiagonal();

    // Declare the Jacobian of the measurements wrt the robot pose
    Eigen::Matrix<double, 2, 3> H; // H = J_e_x

    // Declare some temporaries
    Eigen::Vector2d e, z;               // expectation, innovation
    Eigen::Matrix2d E, Z;               // covs of the above
    Eigen::Matrix<double, 3, 2> K;      // Kalman gain
    manif::SE2Tangentd dx;              // optimal update step, or error-state
    manif::SE2d::Jacobian J_xi_x;       // Jacobian
    Eigen::Matrix<double, 2, 3> J_e_xi; // Jacobian

    //
    //
    // CONFIGURATION DONE



    // DEBUG
    cout << std::setprecision(3) << std::fixed << endl;
    cout << "X STATE:   X     Y   |   THETA  " << endl;
    cout << "-------------------------------------------------" << endl;
    cout << "X init : " << X_simulation.translation().transpose() << " | " << X_simulation.angle() << endl;
    cout << "-------------------------------------------------" << endl;
    // END DEBUG




    // START TEMPORAL LOOP
    //
    //

    // Make 10 steps. Measure one landmark each time
    for (int t = 0; t < 10; t++)
    {
        //// I. Simulation ###############################################################################

        /// simulate noise
        u_noise = (u_sigmas.array() * Eigen::Array<double, 3, 1>::Random()).matrix();   // control noise
        y_noise = n_sigmas.array() * Eigen::Array<double, 2, 1>::Random();              // measurement noise

        u_noisy = u + u_noise;                                      // noisy control

        u_simu  = u;
        u_est   = u_noisy;

        /// first we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        X_simulation = X_simulation + u_simu;                       // overloaded X.rplus(u) = X * exp(u)

        /// then we measure all landmarks - - - - - - - - - - - - - - - - - - - -
        for (int i = 0; i < 3; i++)
        {
            b = landmarks[i];                                       // lmk coordinates in world frame

            y = X_simulation.inverse().act(b);                      // landmark measurement, before adding noise
            y = y + y_noise;                                        // landmark measurement, noisy
            measurements[i] = y;                                    // store for the estimator just below
        }




        //// II. Estimation ###############################################################################

        /// First we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        X = X.plus(u_est, J_x, J_u);                                // X * exp(u), with Jacobians

        P = J_x * P * J_x.transpose() + J_u * U * J_u.transpose();



        // DEBUG
        cout << "X simu : " << X_simulation.translation().transpose() << " | " << X_simulation.angle() << endl;
        cout << "X pred : " << X.translation().transpose() << " | " << X.angle() << endl;
        // END DEBUG



        /// Then we correct using the measurements of the lmks - - - - - - - - -
        for (int i = 0; i < NUMBER_OF_LMKS_TO_MEASURE; i++)
        {
            // landmark
            b = landmarks[i];                           // lmk coordinates in world frame

           // measurement
            y = measurements[i];                        // lmk measurement, noisy

            // expectation
            e = X.inverse(J_xi_x).act(b, J_e_xi);       // note: e = R.tr * ( b - t ), for X = (R,t).
            H = J_e_xi * J_xi_x;                        // note: H = J_e_x = J_e_xi * J_xi_x
            E = H * P * H.transpose();

            // innovation
            z = y - e;
            Z = E + R;

            // Kalman gain
            K = P * H.transpose() * Z.inverse();        // this expands to  K = P * H.tr * ( H * P * H.tr + R).inv

            // Correction step
            dx = K * z;                                 // dx is in the tangent space at X

            // Update
            X = X + dx;                                 // overloaded X.rplus(dx) = X * exp(dx)
            P = P - K * Z * K.transpose();
        }


        // DEBUG
        cout << "X corr : " << X.translation().transpose() << " | " << X.angle() << endl;
        cout << "-------------------------------------------------" << endl;
        // END DEBUG

    }

    //
    //
    // END OF TEMPORAL LOOP. DONE.

    return 0;
}
