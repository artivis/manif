r"""

\file se_2_3_localization.py.

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
3D Robot localization and linear velocity estimation
based on strap-down IMU model and fixed beacons.

---------------------------------------------------------

We consider a robot in 3D space surrounded by a small
number of punctual landmarks or _beacons_.
The robot is assumed to be mounted with an IMU whose
measurements are fed as exogeneous inputs to the system.
The robot is able to measure the location
of the beacons w.r.t its own reference frame.
We assume in this example that the IMU frame coincides with the robot frame.

The robot extended pose X is in SE_2(3) and the beacon positions b_k in R^3,

    X = |    R   p  v|  // position, orientation and linear velocity
        |        1   |
        |           1|

    b_k = (bx_k, by_k, bz_k)    // lmk coordinates in world frame

    alpha_k = (alphax_k, alphay_k, alphaz_k) // linear accelerometer measurements in IMU frame

    omega_k = (omegax_k, omegay_k, omegaz_k) // gyroscope measurements in IMU frame

    g = (0, 0, -9.80665)  // acceleration due to gravity in world frame

Consider robot coordinate frame B and world coordinate frame A.
- p is the position of the origin of the robot frame B with respect to the world frame A
- R is the orientation of the robot frame B with respect to the world frame A
- v is the velocity of the robot frame with respect to the world frame,
           expressed in a frame whose origin coincides with the robot frame,
           oriented similar to the world frame
           (it is equivalent to p_dot in continuous time.
           This is usually called mixed-frame representation
           and is denoted as (B[A] v_AB),
           where B[A] is the mixed frame as described above.
           For reference, please see "Multibody Dynamics Notation" by
           Silvio Traversaro and Alessandro Saccon.
           Link: https://research.tue.nl/en/publications/multibody-dynamics-notation-version-2)
- a is the frame acceleration in mixed-representation (equivalent to p_doubledot in continuous time).
- omega_b as the angular velocity of the robot expressed in the robot frame

The kinematic equations (1) can be written as,
    p <-- p + v dt + 0.5 a dt^2
    R <-- R Exp_SO3(omega_b)
    v <-- v + a dt

However, we would like to express the kinematics equations in the form,
X <-- X * Exp(u)
where, X \in SE_2(3), u \in R^9 and u_hat \in se_2(3)
Note that here input vector u is expressed in the local frame (robot frame).
This can be seen as a motion integration on a manifold defined by the group SE_2(3).

The exponential mapping of SE_2(3) is defined as,
for u = [u_p, u_w, u_v]
    Exp(u) = | Exp_SO3(u_w)   JlSO3(u_w) u_p   JlSO3(u_w) u_v |
             | 0    0    0                 1                0 |
             | 0    0    0                 0                1 |
where, JlSO3 is the left Jacobian of the SO(3) group.

Please see the Appendix C of the paper
"A micro Lie theory for state estimation in robotics",
for the definition of the left Jacobian of SO(3).
Please see the Appendix D of the paper,
for the definition of Exp map for SE(3).
The Exp map of SE_2(3) is a simple extension from the Exp map of SE(3).
Also, please refer to Example 7 of the paper to understand
when and how the left Jacobian of SO(3) appears in the definitions of Exp maps.
The Example 7 illustrates the scenario for SE(3).
We use a direct extension here for SE_2(3).
One can arrive to such a definition by following
the convergent Taylor's series expansion
for the matrix exponential of
the Lie algebra element (Equation 16 of the paper).

As a result of X <-- X * Exp(u), we get (2)
    p <-- p + R JlSO3(u_w) u_p
    R <-- R Exp_SO3(u_w)
    v <-- v + R JlSO3(u_w) u_v

It is important to notice the subtle difference between (1) and (2) here,
which is specifically the influence of the left Jacobian of SO(3) in (2).
The approach in (1) considers the motion integration is done by defining
the exponential map in R3xSO(3)xR3 instead of SE_2(3),
in the sense explored in Example 7 of the Micro Lie theory paper.
It must be noted that as dt tends to 0,
both sets of equations (1) and (2) tend to be the same,
since JlSO3 tends to identity.

Since, (2) exploits the algebra of the SE_2(3) group properly,
we would like to draw a relationship between the sets of equations (2)
and the IMU measurements which will constitute
the exogeneous input vector u \in se_2(3).

Considering R.T as the transpose of R, the IMU measurements are modeled as,
    - linear accelerometer measurements alpha = R.T (a - g) + w_acc
    - gyroscope measurements omega = omega_b + w_omega
Note that the IMU measurements are expressed in the IMU frame
(coincides with the robot frame - assumption).
The IMU measurements are corrupted by noise,
    - w_omega is the additive white noise affecting the gyroscope measurements
    - w_acc is the additive white noise affecting
      the linear accelerometer measurements
It must be noted that we do not consider IMU biases
in the IMU measurement model in this example.

Taking into account all of the above considerations,
the exogenous input vector u (3) becomes,
    u = (u_p, u_w, u_v) where,
    u_w = omega dt
    u_p = (R.T v dt + 0.5 dt^2 (alpha + R.T g)
    u_v = (alpha + R.T g) dt

This choice of input vector allows us to directly use measurements from the IMU
for an unified motion integration involving position,
orientation and linear velocity of the robot using SE_2(3).
Equations (2) and (3) lead us to the following evolution equations,

    p <-- p + JlSO3 R.T v dt + 0.5 JlSO3 (alpha + R.T g) dt^2
    R <-- R Exp_SO3(omega dt)
    v <-- v + JlSO3 (alpha + R.T g) dt

The system propagation noise covariance matrix becomes,
    U = diagonal(0, 0, 0,
                 sigma_omegax^2, sigma_omegay^2, sigma_omegaz^2,
                 sigma_accx^2, sigma_accy^2, sigma_accz^2).

At the arrival of a exogeneous input u, the robot pose is updated
with X <-- X * Exp(u) = X + u.

Landmark measurements are of the range and bearing type,
though they are put in Cartesian form for simplicity.
Their noise n is zero mean Gaussian, and is specified
with a covariances matrix R.
We notice that the SE_2(3) action is the same as a
rigid motion action of SE(3).
This is the action of X \in SE_2(3) on a 3-d point b \in R^3 defined as,
X b = R b + p

Thus, the landmark measurements can be expressed as
a group action on 3d points,
y = h(X,b) = X^-1 * b

    y_k = (brx_k, bry_k, brz_k)    // lmk coordinates in robot frame

We consider the beacons b_k situated at known positions.
We define the extended pose to estimate as X in SE_2(3).
The estimation error dx and its covariance P are expressed
in the tangent space at X.

All these variables are summarized again as follows

    X   : robot's extended pose, SE_2(3)
    u   : robot control input, u = u(X, y_imu) \in se_2(3) with X as state and
          y_imu = [alpha, omega] as IMU readings, see Eq. (3)
    U   : control perturbation covariance
    b_k : k-th landmark position, R^3
    y   : Cartesian landmark measurement in robot frame, R^3
    R   : covariance of the measurement noise

The motion and measurement models are

    X_(t+1) = f(X_t, u) = X_t * Exp ( u )     // motion equation
    y_k     = h(X, b_k) = X^-1 * b_k          // measurement equation

The algorithm below comprises first a simulator to
produce measurements, then uses these measurements
to estimate the state, using a Lie-based error-state Kalman filter.

This file has plain code with only one main() function.
There are no function calls other than those involving `manif`.

Printing simulated state and estimated state together
with an unfiltered state (i.e. without Kalman corrections)
allows for evaluating the quality of the estimates.

A side note: Besides the approach described here in this illustration example,
there are other interesting works like the paper,
The Invariant Extended Kalman filter as a stable observer
(https://arxiv.org/pdf/1410.1465.pdf)
which assume a specific structure for the
system propagation dynamics "f(X_t, u)" (group affine dynamics)
that simplifies the covariance propagation and
enables error dynamics with stronger convergence properties.
"""


from manifpy import SE_2_3, SE_2_3Tangent

import numpy as np
from numpy.linalg import inv


Vector = np.array


def Covariance():
    return np.zeros((SE_2_3.DoF, SE_2_3.DoF))


def Jacobian():
    return np.zeros((SE_2_3.DoF, SE_2_3.DoF))


def random(dim):
    """Random vector Rdim in [-1, 1]."""
    return np.random.uniform([-1]*dim, [1]*dim)


def skew(vec):
    mat = np.zeros((3, 3))
    mat[0, 1] = -vec[2]
    mat[0, 2] = +vec[1]
    mat[1, 0] = +vec[2]
    mat[1, 2] = -vec[0]
    mat[2, 0] = -vec[1]
    mat[2, 1] = +vec[0]
    return np.copy(mat)


def frange(start, stop, step):
    return [
        x*step+start for x in range(
            0,
            round(abs((stop-start)/step)+0.5001),
            int((stop-start)/step < 0)*-2+1
        )
    ]


if __name__ == '__main__':

    # START CONFIGURATION

    NUMBER_OF_LMKS_TO_MEASURE = 5

    # Define the robot pose element and its covariance
    X_simulation = SE_2_3.Identity()
    X = SE_2_3.Identity()
    X_unfiltered = SE_2_3.Identity()
    P = Covariance()
    P[0:3, 0:3] = np.diagflat([0.001, 0.001, 0.001])
    P[3:6, 3:6] = np.diagflat([0.01, 0.01, 0.01])
    P[6:9, 6:9] = np.diagflat([0.001, 0.001, 0.001])

    print("P: ", P)

    # acceleration due to gravity in world frame
    g = Vector([0.0, 0.0, -9.80665])
    dt = 0.01

    alpha_const = Vector([0.1, 0.01, 0.1])  # constant acceleration in IMU frame without gravity compensation
    omega = Vector([0.01, 0.1, 0.0])  # constant angular velocity about x- and y-direction in IMU frame

    # Previous IMU measurements in IMU frame initialized
    # to values expected when stationary
    alpha_prev = alpha_const - X_simulation.rotation().transpose() @ g
    alpha = alpha_const - X_simulation.rotation().transpose() @ g
    omega_prev = Vector([0.0, 0.0, 0.0])

    u_nom = Vector([0.0] * 9)
    u_est = Vector([0.0] * 9)
    u_sigmas = Vector([0.0, 0.0, 0.0, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
    U = np.diagflat(np.square(u_sigmas))

    # Declare the Jacobians of the motion wrt robot and control
    F = Jacobian()      # F = J_x_x + (J_x_u * J_u_x)
    J_x_x = Jacobian()  # d(X * exp(u)) / dX
    J_x_u = Jacobian()  # d(X * exp(u)) / du
    J_u_x = Jacobian()  # du / dX, since u is a state-dependent vector

    # Define five landmarks in R^3
    landmarks = []
    landmarks.append(Vector([2.0,  0.0,  0.0]))
    landmarks.append(Vector([3.0, -1.0, -1.0]))
    landmarks.append(Vector([2.0, -1.0,  1.0]))
    landmarks.append(Vector([2.0,  1.0,  1.0]))
    landmarks.append(Vector([2.0,  1.0, -1.0]))

    # Define the beacon's measurements
    measurements = [Vector([0.0, 0.0, 0.0])] * NUMBER_OF_LMKS_TO_MEASURE

    y_sigmas = Vector([0.01, 0.01, 0.01])
    R = np.diagflat(np.square(y_sigmas))

    # Declare some temporaries
    J_xi_x = Jacobian()
    J_e_xi = np.zeros((SE_2_3.Dim, SE_2_3.DoF))

    # CONFIGURATION DONE

    # pretty print
    np.set_printoptions(precision=3, suppress=True, linewidth=160)

    # DEBUG
    print('X STATE     :    X      Y      Z    TH_x   TH_y   TH_z ')
    print('-------------------------------------------------------')
    print('X initial   : ', X_simulation.log().coeffs())
    print('-------------------------------------------------------')

    # END DEBUG

    # START TEMPORAL LOOP

    # Make 10/dt steps. Measure up to three landmarks each time.
    for t in frange(0, 10, dt):
        # I. Simulation

        # get current simulated state and measurements from previous step
        R_k = X_simulation.rotation()
        v_k = X_simulation.linearVelocity()
        acc_k = alpha_prev + R_k.transpose() @ g

        # input vector
        u_nom[0:3] = dt * R_k.transpose() @ v_k + 0.5 * dt * dt * acc_k
        u_nom[3:6] = dt * omega_prev
        u_nom[6:9] = dt * acc_k

        # simulate noise
        u_noise = u_sigmas * random(SE_2_3.DoF)         # control noise
        u_noisy = u_nom + u_noise                       # noisy control

        u_simu = SE_2_3Tangent(u_nom)
        u_unfilt = SE_2_3Tangent(u_noisy)

        # first we move
        X_simulation = X_simulation + u_simu            # X * exp(u)
        # update expected IMU measurements after moving
        alpha = alpha_const - X_simulation.rotation().transpose() @ g

        # then we measure all landmarks
        for i in range(NUMBER_OF_LMKS_TO_MEASURE):
            b = landmarks[i]                            # lmk coordinates in world frame

            # simulate noise
            y_noise = y_sigmas * random(SE_2_3.Dim)     # measurement noise

            y = X_simulation.inverse().act(b)           # landmark measurement, before adding noise

            y = y + y_noise                             # landmark measurement, noisy
            measurements[i] = y                         # store for the estimator just below

        # II. Estimation

        # get current state estimate to build
        # the state-dependent control vector
        R_k_est = X.rotation()
        v_k_est = X.linearVelocity()
        acc_k_est = alpha_prev + R_k_est.transpose() @ g

        accLin = dt * R_k_est.transpose() @ v_k_est + 0.5 * dt * dt * acc_k_est
        gLin = R_k_est.transpose() @ g * dt
        accLinCross = skew(accLin)
        gCross = skew(gLin)

        u_est[0:3] = accLin
        u_est[3:6] = dt * omega_prev
        u_est[6:9] = dt * acc_k_est

        u_est += u_noise

        # First we move

        X = X.plus(SE_2_3Tangent(u_est), J_x_x, J_x_u)  # X * exp(u), with Jacobians

        # Prepare Jacobian of state-dependent control vector
        J_u_x[0:3, 3:6] = accLinCross
        J_u_x[0:3, 6:9] = np.identity(3) * dt
        J_u_x[6:9, 3:6] = gCross
        F = J_x_x + J_x_u @ J_u_x                 # chain rule for system model Jacobian

        P = F @ P @ F.transpose() + J_x_u @ U @ J_x_u.transpose()

        # Then we correct using the measurements of each lmk
        for i in range(NUMBER_OF_LMKS_TO_MEASURE):
            # landmark
            b = landmarks[i]                      # lmk coordinates in world frame

            # measurement
            y = measurements[i]                   # lmk measurement, noisy

            # expectation
            e = X.inverse(J_xi_x).act(b, J_e_xi)  # note: e = R.tr * ( b - t ), for X = (R,t).
            H = J_e_xi @ J_xi_x                   # Jacobian of the measurements wrt the robot pose. note: H = J_e_x = J_e_xi * J_xi_x
            E = H @ P @ H.transpose()

            # innovation
            z = y - e
            Z = E + R

            # Kalman gain
            K = P @ H.transpose() @ inv(Z)        # K = P * H.tr * ( H * P * H.tr + R).inv

            # Correction step
            dx = K @ z                            # dx is in the tangent space at X

            # Update
            X = X + SE_2_3Tangent(dx)             # overloaded X.rplus(dx) = X * exp(dx)

            P = P - K @ Z @ K.transpose()

        # III. Unfiltered

        # move also an unfiltered version for comparison purposes
        X_unfiltered = X_unfiltered + u_unfilt

        alpha_prev = np.copy(alpha)
        omega_prev = np.copy(omega)

        # IV. Results

        # DEBUG
        print('X simulated : ', X_simulation.log().coeffs().transpose())
        print('X estimated : ', X.log().coeffs().transpose())
        print('X unfilterd : ', X_unfiltered.log().coeffs().transpose())
        print('-------------------------------------------------------')
        # END DEBUG
