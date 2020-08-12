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
 *  adapted from the file se3_localization.cpp:
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
 *  3D Robot localization and linear velocity estimation based on strap-down IMU model and fixed beacons.
 *
 *  ---------------------------------------------------------
 *
 *  We consider a robot in 3D space surrounded by a small
 *  number of punctual landmarks or _beacons_.
 *  The robot is assumed to be mounted with an IMU whose
 *  measurements are fed as exogeneous inputs to the system.
 *  The robot is able to measure the location
 *  of the beacons w.r.t its own reference frame.
 *  We assume in this example that the IMU frame coincides with the robot frame.
 *
 *  The robot extended pose X is in SE_2(3) and the beacon positions b_k in R^3,
 *
 *      X = |    R   p  v|              // position, orientation and linear velocity
 *          |0 0 0   1  0|
 *          |0 0 0   0  1|
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
 * Consider robot coordinate frame B and world coordinate frame A.
 * - p is the position of the origin of the robot frame B with respect to the world frame A
 * - R is the orientation of the robot frame B with respect to the world frame A
 * - v is the velocity of the robot frame with respect to the world frame,
 *            expressed in a frame whose origin coincides with the robot frame, oriented similar to the world frame
 *            (it is equivalent to p_dot in continuous time. This is usually called mixed-frame representation
 *            and is denoted as (B[A] v_AB), where B[A] is the mixed frame as described above.
 *            For reference, please see "Multibody Dynamics Notation" by Silvio Traversaro and Alessandro Saccon.
 *            Link: https://research.tue.nl/en/publications/multibody-dynamics-notation-version-2)
 * - a is the frame acceleration in mixed-representation (equivalent to p_doubledot in continuous time).
 * - omega_b as the angular velocity of the robot expressed in the robot frame
 *
 * The kinematic equations (1) can be written as,
 * p <-- p + v dt + 0.5 a dt^2
 * R <-- R Exp_SO3(omega_b)
 * v <-- v + a dt
 *
 * We would like to express these equations in the form,
 * X <-- X* Exp(u)
 * where, X \in SE_2(3), u \in R^9 and u_hat \in se_2(3)
 * Note that here input vector u is expressed in the local frame (robot frame).
 *
 * The exponential mapping of SE_2(3) is defined as,
 * for u = [u_p, u_w, u_v]
 * Exp(u) = | Exp_SO3(u_w)   JlSO3(u_w) u_p   JlSO3(u_w) u_v |
 *          | 0    0    0                 1                0 |
 *          | 0    0    0                 0                1 |
 * where, JlSO3 is the left Jacobian of the SO(3) group.
 *
 * Please see the Appendix C of the paper "A micro Lie theory for state estimation in robotics",
 * for the definition of the left Jacobian of SO(3).
 * Please see the Appendix D of the paper, for the definition of Exp map for SE(3).
 * The Exp map of SE_2(3) is a simple extension from the Exp map of SE(3).
 * Also, please refer to Example 7 of the paper to understand when and how the left Jacobian of SO(3)
 * appears in the definitions of Exp maps. The Example 7 illustrates the scenario for SE(3).
 * We use a direct extension here for SE_2(3).
 * One can arrive to such a definition by following the convergent Taylor's series expansion
 * for the matrix exponential of the Lie algebra element (Equation 16 of the paper).
 *
 * As a result of X <-- X * Exp(u), we get (2)
 * p <-- p + R JlSO3(u_w) u_p
 * R <-- R Exp_SO3(u_w)
 * v <-- v + R JlSO3(u_w) u_v
 *
 * We would like to draw a relationship between the sets of equations (1) and (2).
 * Additionally, we would also like to make use of the IMU measurements as exogeneous inputs.
 * Considering R.T as the transpose of R, the IMU measurements are modeled as,
 *    - linear accelerometer measurements alpha = R.T (a - g) + w_acc
 *    - gyroscope measurements omega = omega_b + w_omega
 * Note that the IMU measurements are expressed in the IMU frame (coincides with the robot frame - assumption).
 * The IMU measurements are corrupted by noise,
 *    - w_omega is the additive white noise affecting the gyroscope measurements
 *    - w_acc is the additive white noise affecting the linear accelerometer measurements
 *
 * Taking into account all of the above considerations, the exogenous input vector u (3) becomes,
 *   u = (u_p, u_w, u_v) where,
 *   u_w = omega dt
 *   Jinv = inv(JlSO3(u_w)) is the left Jacobian inverse of SO(3) Lie group
 *   u_p = Jinv * (R.T v dt + 0.5 dt^2 (alpha + R.T g))
 *   u_v = Jinv * ((alpha + R.T g) dt)
 *
 * To cross verify the equations, substituting (3) in (2) will give us (1).
 * This choice of input vector allows us to directly use measurements from the IMU
 * for an unified motion integration involving position, orientation and linear velocity of the robot.
 *
 * The system propagation noise covariance amtrix becomes,
 *    U = diagonal(0, 0, 0, sigma_omegax^2, sigma_omegay^2, sigma_omegaz^2, sigma_accx^2, sigma_accy^2, sigma_accz^2).
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
 *    u   : robot control input, (Jinv * (R.T v dt + 0.5 dt^2 (alpha + R.T g)), omega dt, Jinv * ((alpha + R.T g) dt))
 *    U   : control perturbation covariance
 *    b_k : k-th landmark position, R^3
 *    y   : Cartesian landmark measurement in robot frame, R^3
 *    R   : covariance of the measurement noise
 *
 *  The motion and measurement models are
 *
 *    X_(t+1) = f(X_t, u) = X_t * Exp ( u )     // motion equation
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
 *
 * A side note: Besides the approach described here in this illustration example,
 * there are other interesting works like the paper,
 * The Invariant Extended Kalman filter as a stable observer (https://arxiv.org/pdf/1410.1465.pdf)
 * which assume a specific structure for the system propagation dynamics "f(X_t, u)" (group affine dynamics)
 * that simplifies the covariance propagation and enables error dynamics with stronger convergence properties.
 *
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
    alpha << 10.0, 0.0, 9.80665; // constant acceleration along x-direction
    omega << 1, 0.1, 0; // constant angular velocity about x- and y-direction
    // Previous IMU measurements in IMU frame initialized to values expected when stationary
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

        /// get current state and measurements from previous step
        auto R_k = X_simulation.rotation();
        auto v_k = X_simulation.linearVelocity();
        auto acc_k = alpha_prev + R_k.transpose()*g;
        manif::SO3Tangentd Rtan = dt*omega_prev;
        auto Jinv = Rtan.ljacinv();

        /// state dependent velocity vector
        u_nom << Jinv*(dt*(R.transpose())*v_k + 0.5*dt*dt*acc_k),  dt*omega_prev, Jinv*dt*acc_k;

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
            measurements[i] << y, 1, 0;                     // store for the estimator just below
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
