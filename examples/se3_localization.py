r"""

\file se3_localization.py.

Created on: Jan 11, 2021
\author: Jeremie Deray

---------------------------------------------------------
This file is:
(c) 2021 Jeremie Deray

This file is part of `manif`, a C++ template-only library
for Lie theory targeted at estimation for robotics.
Manif is:
(c) 2021 Jeremie Deray
---------------------------------------------------------

---------------------------------------------------------
Demonstration example:

    3D Robot localization based on fixed beacons.

    See se3_sam.py for a more advanced example performing smoothing and mapping.
---------------------------------------------------------

This demo corresponds to the 3D version of the application
in chapter V, section A, in the paper Sola-18,
[https://arxiv.org/abs/1812.01537].

The following is an abstract of the content of the paper.
Please consult the paper for better reference.

We consider a robot in 3D space surrounded by a small
number of punctual landmarks or _beacons_.
The robot receives control actions in the form of axial
and angular velocities, and is able to measure the location
of the beacons w.r.t its own reference frame.

The robot pose X is in SE(3) and the beacon positions b_k in R^3,

    X = |  R   t |              // position and orientation
        |  0   1 |

    b_k = (bx_k, by_k, bz_k)    // lmk coordinates in world frame

The control signal u is a twist in se(3) comprising longitudinal
velocity vx and angular velocity wz, with no other velocity
components, integrated over the sampling time dt.

    u = (vx*dt, 0, 0, 0, 0, w*dt)

The control is corrupted by additive Gaussian noise u_noise,
with covariance

    Q = diagonal(sigma_x^2, sigma_y^2, sigma_z^2, sigma_roll^2, sigma_pitch^2, sigma_yaw^2).

This noise accounts for possible lateral and rotational slippage
through a non-zero values of sigma_y, sigma_z, sigma_roll and sigma_pitch.

At the arrival of a control u, the robot pose is updated
with X <-- X * Exp(u) = X + u.

Landmark measurements are of the range and bearing type,
though they are put in Cartesian form for simplicity.
Their noise n is zero mean Gaussian, and is specified
with a covariances matrix R.
We notice the rigid motion action y = h(X,b) = X^-1 * b
(see appendix D),

    y_k = (brx_k, bry_k, brz_k)    // lmk coordinates in robot frame

We consider the beacons b_k situated at known positions.
We define the pose to estimate as X in SE(3).
The estimation error dx and its covariance P are expressed
in the tangent space at X.

All these variables are summarized again as follows

    X   : robot pose, SE(3)
    u   : robot control, (v*dt; 0; 0; 0; 0; w*dt) in se(3)
    Q   : control perturbation covariance
    b_k : k-th landmark position, R^3
    y   : Cartesian landmark measurement in robot frame, R^3
    R   : covariance of the measurement noise

The motion and measurement models are

    X_(t+1) = f(X_t, u) = X_t * Exp ( w )     // motion equation
    y_k     = h(X, b_k) = X^-1 * b_k          // measurement equation

The algorithm below comprises first a simulator to
produce measurements, then uses these measurements
to estimate the state, using a Lie-based error-state Kalman filter.

This file has plain code with only one main() function.
There are no function calls other than those involving `manif`.

Printing simulated state and estimated state together
with an unfiltered state (i.e. without Kalman corrections)
allows for evaluating the quality of the estimates.
"""


from manifpy import SE3, SE3Tangent

import numpy as np
from numpy.linalg import inv


Vector = np.array


def Covariance():
    return np.zeros((SE3.DoF, SE3.DoF))


def Jacobian():
    return np.zeros((SE3.DoF, SE3.DoF))


if __name__ == '__main__':

    # START CONFIGURATION

    NUMBER_OF_LMKS_TO_MEASURE = 5

    # Define the robot pose element and its covariance
    X_simulation = SE3.Identity()
    X = SE3.Identity()
    X_unfiltered = SE3.Identity()
    P = Covariance()

    u_nom = Vector([0.1, 0.0, 0.0, 0.0, 0.0, 0.05])
    u_sigmas = Vector([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    U = np.diagflat(np.square(u_sigmas))

    # Declare the Jacobians of the motion wrt robot and control
    J_x = Jacobian()
    J_u = Jacobian()

    # Define five landmarks in R^3
    landmarks = []
    landmarks.append(Vector([2.0,  0.0,  0.0]))
    landmarks.append(Vector([3.0, -1.0, -1.0]))
    landmarks.append(Vector([2.0, -1.0,  1.0]))
    landmarks.append(Vector([2.0,  1.0,  1.0]))
    landmarks.append(Vector([2.0,  1.0, -1.0]))

    # Define the beacon's measurements
    measurements = [Vector([0, 0, 0])] * NUMBER_OF_LMKS_TO_MEASURE

    y_sigmas = Vector([0.01, 0.01, 0.01])
    R = np.diagflat(np.square(y_sigmas))

    # Declare some temporaries
    J_xi_x = Jacobian()
    J_e_xi = np.zeros((SE3.Dim, SE3.DoF))

    # CONFIGURATION DONE

    # pretty print
    np.set_printoptions(precision=3, suppress=True)

    # DEBUG
    print('X STATE     :    X      Y      Z    TH_x   TH_y   TH_z ')
    print('-------------------------------------------------------')
    print('X initial   : ', X_simulation.log().coeffs())
    print('-------------------------------------------------------')
    # END DEBUG

    # START TEMPORAL LOOP

    # Make 10 steps. Measure up to three landmarks each time.
    for t in range(10):
        # I. Simulation

        # simulate noise
        u_noise = u_sigmas * np.random.rand(SE3.DoF)      # control noise
        u_noisy = u_nom + u_noise                         # noisy control

        u_simu = SE3Tangent(u_nom)
        u_est = SE3Tangent(u_noisy)
        u_unfilt = SE3Tangent(u_noisy)

        # first we move
        X_simulation = X_simulation + u_simu              # overloaded X.rplus(u) = X * exp(u)

        # then we measure all landmarks
        for i in range(NUMBER_OF_LMKS_TO_MEASURE):
            b = landmarks[i]                              # lmk coordinates in world frame

            # simulate noise
            y_noise = y_sigmas * np.random.rand(SE3.Dim)  # measurement noise

            y = X_simulation.inverse().act(b)             # landmark measurement, before adding noise

            y = Vector(y) + y_noise                       # landmark measurement, noisy
            measurements[i] = y                           # store for the estimator just below

        # II. Estimation

        # First we move

        X = X.plus(u_est, J_x, J_u)                       # X * exp(u), with Jacobians

        P = J_x * P * J_x.transpose() + J_u * U * J_u.transpose()

        # Then we correct using the measurements of each lmk
        for i in range(NUMBER_OF_LMKS_TO_MEASURE):
            # landmark
            b = landmarks[i]                              # lmk coordinates in world frame

            # measurement
            y = measurements[i]                           # lmk measurement, noisy

            # expectation
            e = X.inverse(J_xi_x).act(b, J_e_xi)          # note: e = R.tr * ( b - t ), for X = (R,t).
            e = Vector(e)
            H = J_e_xi @ J_xi_x                           # Jacobian of the measurements wrt the robot pose. note: H = J_e_x = J_e_xi * J_xi_x
            E = H @ P @ H.transpose()

            # innovation
            z = y - e
            Z = E + R

            # Kalman gain
            K = P @ H.transpose() @ inv(Z)                # K = P * H.tr * ( H * P * H.tr + R).inv

            # Correction step
            dx = K @ z                                    # dx is in the tangent space at X

            # Update
            X = X + SE3Tangent(dx)                        # overloaded X.rplus(dx) = X * exp(dx)
            P = P - K @ Z @ K.transpose()

        # III. Unfiltered

        # move also an unfiltered version for comparison purposes
        X_unfiltered = X_unfiltered + u_unfilt

        # IV. Results

        # DEBUG
        print('X simulated : ', X_simulation.log().coeffs().transpose())
        print('X estimated : ', X.log().coeffs().transpose())
        print('X unfilterd : ', X_unfiltered.log().coeffs().transpose())
        print('-------------------------------------------------------')
        # END DEBUG
