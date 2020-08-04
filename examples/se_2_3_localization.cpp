/**
 * \file se_2_3_localization.cpp
 *
 *  Created on: Aug 03, 2020
 *     \author: prashanthr05
 *
 *  ---------------------------------------------------------
 *  This file is:
 *  (c) 2020 Prashanth Ramadoss @ DIC-IIT, Genova, Italy
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
 *  3D Robot localization and linear velocity estimation based on strap-down IMU model and fixed beacons.
 *
 *  ---------------------------------------------------------
 *
 *  We consider a robot in 3D space surrounded by a small
 *  number of punctual landmarks or _beacons_.
 *  The robot is assumed to be mounted with an IMU whose
 *  measurements are fed as exogeneous inputs to the system,
 *  and is able to measure the location
 *  of the beacons w.r.t its own reference frame.
 *  We assume in this example that the IMU frame coincides with the robot frame.
 *
 *  The robot extended pose X is in SE_2(3) and the beacon positions b_k in R^3,
 *
 *      X = |  R   t  v|              // position, orientation and linear velocity
 *          |  0   1  0|
 *          |  0   0  1|
 *
 *      b_k = (bx_k, by_k, bz_k)    // lmk coordinates in world frame
 *
 *      alpha_k = (alphax_k, alphay_k, alphaz_k) // linear accelerometer measurements in IMU frame
 *
 *      omega_k = (omegax_k, omegay_k, omegaz_k) // gyroscope measurements in IMU frame
 *
 *      g = (0, 0, -9.80665)  // acceleration due to gravity in world frame
 *
 * Note that the linear velocity v is expressed in a frame whose origin coincides with
 * the robot frame but has a orientation which is same as the world frame
 *
 *  The discrete-time system propagation can be written as,
 *   p_{k+1} = p_{k} + R_{k} (R_{k}.T v_{k} dt + 0.5*dt*dt*acc_{k})
 *   R_{k+1} = R_{k} Exp_SO3(omega_{k} dt + w_omega)
 *   v_{k+1} = v_{k} + R_{k} (acc_{k} dt + w_acc)
 *
 * Note that these equations are modified to obey the group composition rules.
 *
 * where,
 *        - R_{k}.T is the transpose of R_{k}
 *        - acc_{k} = alpha_{k} + R_{k}.T g
 *        - w_omega is the additive white noise affecting the gyroscope measurements
 *        - w_acc is the additive white noise affecting the linear accelerometer measurements
 *
 *    Q = diagonal(0_{3x1}, 0_{3x1}, 0_{3x1}, sigma_omegax^2, sigma_omegay^2, sigma_omegaz^2, sigma_accx^2, sigma_accy^2, sigma_accz^2).
 *
 *  The exogenous input u becomes,
 *   u = (R_{k}.T v dt + 0.5*dt*dt*acc_{k}, omega_{k} dt, acc_{k} dt)
 *
 * Note that all the elements of the input vector u belong in the IMU frame/robot frame.
 *
 *
 *  At the arrival of a exogeneous input u, the robot pose is updated
 *  with X <-- X * Exp(u) = X + u.
 *
 *  Landmark measurements are of the range and bearing type,
 *  though they are put in Cartesian form for simplicity.
 *  Their noise n is zero mean Gaussian, and is specified
 *  with a covariances matrix R.
 *  We notice the rigid motion action [y 0 0]' = h(X,b) = X^-1 * [b 1 0]'
 *
 *      y_k = (brx_k, bry_k, brz_k)    // lmk coordinates in robot frame
 *
 *  We consider the beacons b_k situated at known positions.
 *  We define the extended pose to estimate as X in SE_2(3).
 *  The estimation error dx and its covariance P are expressed
 *  in the tangent space at X.
 *
 *  All these variables are summarized again as follows
 *
 *    X   : robot's extended pose, SE_2(3)
 *    u   : robot control input, (R_{k}.T v dt + 0.5*dt*dt*acc_{k}, omega_{k} dt, acc_{k} dt)
 *    Q   : control perturbation covariance
 *    b_k : k-th landmark position, R^3
 *    y   : Cartesian landmark measurement in robot frame, R^3
 *    R   : covariance of the measurement noise
 *
 *  The motion and measurement models are
 *
 *    X_(t+1) = f(X_t, u) = X_t * Exp ( w )     // motion equation
 *    [y_k; 1; 0]     = h(X, b_k) = X^-1 * [b_k; 1; 0]          // measurement equation
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

#include "manif/SE_2_3.h"

#include <Eigen/Dense>

#include <vector>

#include <iostream>
#include <iomanip>

using std::cout;
using std::endl;

using namespace Eigen;

typedef Array<double, 3, 1> Array3d;
typedef Matrix<double, 5, 1> Vector5d;
typedef Array<double, 6, 1> Array6d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Array<double, 9, 1> Array9d;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 9, 9> Matrix9d;


int main()
{
    // START CONFIGURATION
    //
    //
    const int NUMBER_OF_LMKS_TO_MEASURE = 5;

    // Define the robot extended pose element and its covariance
    manif::SE_2_3d X, X_simulation, X_unfiltered;
    Matrix9d    P;

    X_simulation.setIdentity();
    X.setIdentity();
    X_unfiltered.setIdentity();
    P.setZero();
    P.block<3, 3>(0,0) = 0.01*Eigen::Matrix3d::Identity();

    // acceleration due to gravity in world frame
    Vector3d g;
    g << 0, 0, -9.80665;
    const double dt = 0.1;

    // IMU measurements in IMU frame
    Vector3d alpha, omega, alpha_prev, omega_prev;
    alpha << 10.0, 0.0, 9.80665;
    omega << 1, 0.1, 0;
    alpha_prev << 0.0, 0.0, 9.80665;
    omega_prev << 0, 0, 0;

    // Define a control vector and its noise and covariance
    manif::SE_2_3Tangentd  u_simu, u_est, u_unfilt;
    Vector9d            u_nom, u_noisy, u_noise;
    Array9d             u_sigmas;
    Matrix9d            U;

    u_sigmas << 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    U        = (u_sigmas * u_sigmas).matrix().asDiagonal();

    // Declare the Jacobians of the motion wrt robot and control
    manif::SE_2_3d::Jacobian J_x, J_u;

    // Define five landmarks in R^3
    Vector3d b0, b1, b2, b3, b4, b;
    b0 << 2.0,  0.0,  0.0;
    b1 << 3.0, -1.0, -1.0;
    b2 << 2.0, -1.0,  1.0;
    b3 << 2.0,  1.0,  1.0;
    b4 << 2.0,  1.0, -1.0;
    std::vector<Vector3d> landmarks;
    landmarks.push_back(b0);
    landmarks.push_back(b1);
    landmarks.push_back(b2);
    landmarks.push_back(b3);
    landmarks.push_back(b4);

    // Define the beacon's measurements
    Vector3d                y, y_noise;
    Array3d                 y_sigmas;
    Matrix3d                R;
    std::vector<Vector5d>   measurements(landmarks.size());

    y_sigmas << 0.01, 0.01, 0.01;
    R        = (y_sigmas * y_sigmas).matrix().asDiagonal();

    // Declare the Jacobian of the measurements wrt the robot pose
    Matrix<double, 3, 9>    H;      // H = J_e_x

    // Declare some temporaries
    Vector3d                e, z;   // expectation, innovation
    Matrix3d                E, Z;   // covariances of the above
    Matrix<double, 9, 3>    K;      // Kalman gain
    manif::SE_2_3Tangentd      dx;     // optimal update step, or error-state
    manif::SE_2_3d::Jacobian   J_xi_x; // Jacobian is typedef Matrix
    Matrix<double, 3, 9>       J_e_xi; // Jacobian

    //
    //
    // CONFIGURATION DONE



    // DEBUG
    cout << std::fixed   << std::setprecision(3) << std::showpos << endl;
    cout << "X STATE     :    X      Y      Z    TH_x   TH_y   TH_z     V_x   V_y    V_z" << endl;
    cout << "---------------------------------------------------------------------------" << endl;
    cout << "X initial   : " << X_simulation.log().coeffs().transpose() << endl;
    cout << "---------------------------------------------------------------------------" << endl;
    // END DEBUG




    // START TEMPORAL LOOP
    //
    //

    // Make 10 steps. Measure up to three landmarks each time.
    for (int t = 0; t < 10; t++)
    {
        //// I. Simulation ###############################################################################

        /// get current state
        auto R_k = X_simulation.rotation();
        auto v_k = X_simulation.linearVelocity();
        auto acc_k = alpha_prev + R_k.transpose()*g;

        /// state dependent velocity vector
        u_nom << (dt*(R.transpose())*v_k + 0.5*dt*dt*acc_k),  dt*omega_prev, dt*acc_k;

        /// simulate noise
        u_noise = u_sigmas * Array9d::Random();             // control noise
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
            y_noise = y_sigmas * Array3d::Random();         // measurement noise

            y = X_simulation.inverse().act(b);              // landmark measurement, before adding noise
            y = y + y_noise;                                // landmark measurement, noisy
            measurements[i] << y, 1, 0;                            // store for the estimator just below
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
            y = measurements[i].segment<3>(0);                            // lmk measurement, noisy

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

        alpha_prev = alpha;
        omega_prev = omega;


        //// IV. Results ##############################################################################

        // DEBUG
        cout << "X simulated : " << X_simulation.log().coeffs().transpose() << endl;
        cout << "X estimated : " << X.log().coeffs().transpose() << endl;
        cout << "X unfilterd : " << X_unfiltered.log().coeffs().transpose() << endl;
        cout << "---------------------------------------------------------------------------" << endl;
        // END DEBUG

    }

    //
    //
    // END OF TEMPORAL LOOP. DONE.

    return 0;
}
