/*
 * se2_localization.cpp
 *
 *  Created on: Dec 10, 2018
 *      Author: jsola
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
    manif::SE2Tangentd u, u_noisy, u_simu, u_est;
    Eigen::Vector3d u_sigmas, u_noise;
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
    Eigen::Matrix2d N;
    Eigen::Vector2d n_sigmas, y_noise; 
    std::vector<Eigen::Vector2d> measurements(landmarks.size());

    n_sigmas << 0.01, 0.01;
    N = (n_sigmas.array() * n_sigmas.array()).matrix().asDiagonal();

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

        u_noisy = u + manif::SE2Tangentd(u_noise);                                      // noisy control

        u_simu = u;
        u_est  = u_noisy;

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
            Z = E + N;

            // Kalman gain
            K = P * H.transpose() * Z.inverse();        // this expands to  K = P * H.tr * ( H * P * H.tr + N).inv

            // Correction step
            dx = (K * z).eval();                        // eval() is here because the `=` does not accept expressions as right-value.

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
