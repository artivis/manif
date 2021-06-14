r"""

\file se3_sam.py.

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

    3D smoothing and mapping (SAM).

    See se2_sam.py          for a 2D version of this example.
    See se3_localization.py for a simpler localization example using EKF.
------------------------------------------------------------

This demo corresponds to the 3D version of the application
in chapter V, section B, in the paper Sola-18,
[https://arxiv.org/abs/1812.01537].

The following is an abstract of the content of the paper.
Please consult the paper for better reference.


We consider a robot in 3D space surrounded by a small
number of punctual landmarks or _beacons_.
The robot receives control actions in the form of axial
and angular velocities, and is able to measure the location
of the beacons w.r.t its own reference frame.

The robot pose X_i is in SE(3) and the beacon positions b_k in R^3,

    X_i = |  R_i   t_i |        // position and orientation
          |   0     1  |

    b_k = (bx_k, by_k, bz_k)    // lmk coordinates in world frame

The control signal u is a twist in se(3) comprising longitudinal
velocity vx and angular velocity wz, with no other velocity
components, integrated over the sampling time dt.

    u = (vx*dt, 0, 0, 0, 0, w*dt)

The control is corrupted by additive Gaussian noise u_noise,
with covariance

    Q = diagonal(
        sigma_x^2, sigma_y^2, sigma_z^2,
        sigma_roll^2, sigma_pitch^2, sigma_yaw^2
    ).

This noise accounts for possible lateral slippage
through non-zero values of sigma_y, sigma_z, sigma_roll and sigma_pitch.

At the arrival of a control u, a new robot pose is created at

    X_j = X_i * Exp(u) = X_i + u.

This new pose is then added to the graph.

Landmark measurements are of the range and bearing type,
though they are put in Cartesian form for simplicity,

    y = (yx, yy, yz)        // lmk coordinates in robot frame

Their noise n is zero mean Gaussian, and is specified
with a covariances matrix R.
We notice the rigid motion action y_ik = h(X_i,b_k) = X_i^-1 * b_k
(see appendix D).

The world comprises 5 landmarks.
Not all of them are observed from each pose.
A set of pairs pose--landmark is created to establish which
landmarks are observed from each pose.
These pairs can be observed in the factor graph, as follows.

The factor graph of the SAM problem looks like this:

            ------- b1
    b3    /         |
    |   /       b4  |
    | /       /    \|
    X0 ---- X1 ---- X2
    | \   /   \   /
    |   b0      b2
    *

where:
    - X_i are SE3 robot poses
    - b_k are R^3 landmarks or beacons
    - * is a pose prior to anchor the map and make the problem observable
    - segments indicate measurement factors:
        - motion measurements from X_i to X_j
        - landmark measurements from X_i to b_k
        - absolute pose measurement from X0 to * (the origin of coordinates)

We thus declare 9 factors pose---landmark, as follows:

    poses ---  lmks
      x0  ---  b0
      x0  ---  b1
      x0  ---  b3
      x1  ---  b0
      x1  ---  b2
      x1  ---  b4
      x2  ---  b1
      x2  ---  b2
      x2  ---  b4


The main variables are summarized again as follows

    Xi  : robot pose at time i, SE(3)
    u   : robot control, (v*dt; 0; w*dt) in se(3)
    Q   : control perturbation covariance
    b   : landmark position, R^3
    y   : Cartesian landmark measurement in robot frame, R^3
    R   : covariance of the measurement noise


We define the state to estimate as a manifold composite:

    X in  < SE3, SE3, SE3, R^3, R^3, R^3, R^3, R^3 >

    X  =  <  X0,  X1,  X2,  b0,  b1,  b2,  b3,  b4 >

The estimation error dX is expressed
in the tangent space at X,

    dX in  < se3, se3, se3, R^3, R^3, R^3, R^3, R^3 >
        ~  < R^6, R^6, R^6, R^3, R^3, R^3, R^3, R^3 > = R^33

    dX  =  [ dx0, dx1, dx2, db0, db1, db2, db3, db4 ] in R^33

with
    dx_i: pose error in se(3) ~ R^6
    db_k: landmark error in R^3


The prior, motion and measurement models are

    - for the prior factor:
        p_0     = X_0

    - for the motion factors - motion expectation equation:
        d_ij    = X_j (-) X_i = log(X_i.inv * X_j)

    - for the measurement factors - measurement expectation equation:
        e_ik    = h(X_i, b_k) = X_i^-1 * b_k


The algorithm below comprises first a simulator to
produce measurements, then uses these measurements
to estimate the state, using a graph representation
and Lie-based non-linear iterative least squares solver
that uses the pseudo-inverse method.

This file has plain code with only one main() function.
There are no function calls other than those involving `manif`.

Printing the prior state (before solving) and posterior state (after solving),
together with a ground-truth state defined by the simulator
allows for evaluating the quality of the estimates.

This information is complemented with the evolution of
the optimizer's residual and optimal step norms. This allows
for evaluating the convergence of the optimizer.
"""


from manifpy import SE3, SE3Tangent

import numpy as np
from numpy.linalg import inv, norm


Vector = np.array


def Jacobian():
    return np.zeros((SE3.DoF, SE3.DoF))


def random(dim):
    """Random vector Rdim in [-1, 1]."""
    return np.random.uniform([-1]*dim, [1]*dim)


if __name__ == '__main__':

    print()
    print('3D Smoothing and Mapping. 3 poses, 5 landmarks.')
    print('-----------------------------------------------')
    np.set_printoptions(precision=3, suppress=True)

    # START CONFIGURATION

    # some experiment constants
    DoF = SE3.DoF
    Dim = SE3.Dim

    NUM_POSES = 3
    NUM_LMKS = 5
    NUM_FACTORS = 9
    NUM_STATES = NUM_POSES * DoF + NUM_LMKS * Dim
    NUM_MEAS = NUM_POSES * DoF + NUM_FACTORS * Dim
    MAX_ITER = 20  # for the solver

    # Define the robot pose element
    Xi = SE3.Identity()
    X_simu = SE3.Identity()

    u_nom = Vector([0.1, 0.0, 0.0, 0.0, 0.0, 0.05])
    u_sigmas = Vector([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
    Q = np.diagflat(np.square(u_sigmas))
    W = np.diagflat(1./u_sigmas)  # this is Q^(-T/2)

    # Declare the Jacobians of the motion wrt robot and control
    J_x = Jacobian()
    J_u = Jacobian()

    controls = []

    # Define five landmarks in R^2
    landmarks = [0] * NUM_LMKS
    landmarks_simu = [
        Vector([3.0,  0.0,  0.0]),
        Vector([2.0, -1.0, -1.0]),
        Vector([2.0, -1.0,  1.0]),
        Vector([2.0,  1.0,  1.0]),
        Vector([2.0,  1.0, -1.0]),
    ]

    y_sigmas = Vector([0.001, 0.001, 0.001])
    R = np.diagflat(np.square(y_sigmas))
    S = np.diagflat(1./y_sigmas)  # this is R^(-T/2)

    # Declare some temporaries
    J_d_xi = Jacobian()
    J_d_xj = Jacobian()
    J_ix_x = Jacobian()
    J_r_p0 = Jacobian()
    J_e_ix = np.zeros((Dim, DoF))
    J_e_b = np.zeros((Dim, Dim))
    r = np.zeros(NUM_MEAS)
    J = np.zeros((NUM_MEAS, NUM_STATES))

    r"""
    The factor graph of the SAM problem looks like this:

                ------- b1
        b3    /         |
        |   /       b4  |
        | /       /    \|
        X0 ---- X1 ---- X2
        | \   /   \   /
        |   b0      b2
        *

    where:
        - Xi are poses
        - bk are landmarks or beacons
        - * is a pose prior to anchor the map and make the problem observable

    Define pairs of nodes for all the landmark measurements
    There are 3 pose nodes [0..2] and 5 landmarks [0..4].
    A pair pose -- lmk means that the lmk was measured from the pose
    Each pair declares a factor in the factor graph
    We declare 9 pairs, or 9 factors, as follows:
    """

    # 0-0,1,3 | 1-0,2,4 | 2-1,2,4
    pairs = [[0, 1, 3], [0, 2, 4], [1, 2, 4]]

    # Define the beacon's measurements
    measurements = {
        0: {0: 0, 1: 0, 3: 0},
        1: {0: 0, 2: 0, 4: 0},
        2: {1: 0, 2: 0, 4: 0}
    }

    # END CONFIGURATION

    # Simulator

    poses_simu = []
    poses_simu.append(X_simu)
    poses = []
    poses.append(Xi + (SE3Tangent.Random()*0.1))  # use very noisy priors

    # Make 10 steps. Measure up to three landmarks each time.
    for i in range(NUM_POSES):

        # make measurements
        for k in pairs[i]:

            # simulate measurement
            b = landmarks_simu[k]             # lmk coordinates in world frame
            y_noise = y_sigmas * random(Dim)  # measurement noise
            y = X_simu.inverse().act(b)       # landmark measurement, before adding noise

            # add noise and compute prior lmk from prior pose
            measurements[i][k] = y + y_noise  # store noisy measurements
            b = Xi.act(y + y_noise)           # mapped landmark with noise
            landmarks[k] = b + random(Dim)    # use noisy priors

        # make motions
        # do not make the last motion since we're done after 3rd pose
        if i < NUM_POSES - 1:

            # move simulator, without noise
            X_simu = X_simu + SE3Tangent(u_nom)

            # move prior, with noise
            u_noise = u_sigmas * random(DoF)

            Xi = Xi + SE3Tangent(u_nom + u_noise)

            # store
            poses_simu.append(X_simu)
            poses.append(Xi + (SE3Tangent.Random()*0.1))  # use noisy priors
            controls.append(u_nom + u_noise)

    # Estimator

    # DEBUG INFO
    print('prior')
    for X in poses:
        print('pose  : ', X.translation().transpose(), ' ', X.log().coeffs()[3:])
    for l in landmarks:
        print('lmk : ', l.transpose())
    print('-----------------------------------------------')

    # iterate
    for iteration in range(MAX_ITER):

        # Clear residual vector and Jacobian matrix
        r.fill(0)
        J.fill(0)

        row = 0
        col = 0

        """
        1. evaluate prior factor

        NOTE (see Chapter 2, Section E, of Sola-18):

        To compute any residual, we consider the following variables:
            r: residual
            e: expectation
            y: prior specification 'measurement'
            W: sqrt information matrix of the measurement noise.

        In case of a non-trivial prior measurement, we need to consider
        the nature of it: is it a global or a local specification?

        When prior information `y` is provided in the global reference,
        we need a left-minus operation (.-) to compute the residual.
        This is usually the case for pose priors, since it is natural
        to specify position and orientation wrt a global reference,

            r = W * (e (.-) y)
              = W * (e * y.inv).log()

        When `y` is provided as a local reference,
        then right-minus (-.) is required,

            r = W * (e (-.) y)
              = W * (y.inv * e).log()

        Notice that if y = Identity()
        then local and global residuals are the same.


        Here, expectation, measurement and info matrix are trivial,
        as follows

        expectation
            e = poses[0];            // first pose

        measurement
            y = SE3d::Identity()     // robot at the origin

        info matrix:
            W = I                    // trivial

        residual uses left-minus since reference measurement is global
            r = W * (poses[0] (.-) measurement)
              = log(poses[0] * Id.inv) = poses[0].log()

        Jacobian matrix :
            J_r_p0 = Jr_inv(log(poses[0]))         // see proof below

            Proof: Let p0 = poses[0] and y = measurement.
                   We have the partials

                    J_r_p0 = W^(T/2) * d(log(p0 * y.inv)/d(poses[0])

            with W = i and y = I.
            Since d(log(r))/d(r) = Jr_inv(r) for any r in the Lie algebra,
            we have
                J_r_p0 = Jr_inv(log(p0))

        residual and Jacobian.
        Notes:
          We have residual = expectation - measurement,
          in global tangent space
          We have the Jacobian in J_r_p0 = J[row:row+DoF, col:col+DoF]
        """

        r[row:row+DoF] = poses[0].lminus(SE3.Identity(), J_r_p0).coeffs()

        J[row:row+DoF, col:col+DoF] = J_r_p0

        row += DoF

        # loop poses
        for i in range(NUM_POSES):

            # 2. evaluate motion factors
            # do not make the last motion since we're done after 3rd pose
            if i < NUM_POSES - 1:

                j = i + 1  # this is next pose's id

                # recover related states and data
                Xi = poses[i]
                Xj = poses[j]
                u = SE3Tangent(controls[i])

                # expectation
                # (use right-minus since motion measurements are local)
                d = Xj.rminus(Xi, J_d_xj, J_d_xi)  # expected motion = Xj (-) Xi

                # residual
                r[row:row+DoF] = W @ (d - u).coeffs()  # residual

                # Jacobian of residual wrt first pose
                col = i * DoF
                J[row:row+DoF, col:col+DoF] = W @ J_d_xi

                # Jacobian of residual wrt second pose
                col = j * DoF
                J[row:row+DoF, col:col+DoF] = W @ J_d_xj

                # advance rows
                row += DoF

            # 3. evaluate measurement factors
            for k in pairs[i]:

                # recover related states and data
                Xi = poses[i]
                b = landmarks[k]
                y = measurements[i][k]

                # expectation
                e = Xi.inverse(J_ix_x).act(b, J_e_ix, J_e_b)  # expected measurement = Xi.inv * bj
                J_e_x = J_e_ix @ J_ix_x                       # chain rule

                # residual
                r[row:row+Dim] = S @ (e - y)

                # Jacobian of residual wrt pose
                col = i * DoF
                J[row:row+Dim, col:col+DoF] = S @ J_e_x

                # Jacobian of residual wrt lmk
                col = NUM_POSES * DoF + k * Dim
                J[row:row+Dim, col:col+Dim] = S @ J_e_b

                # advance rows
                row += Dim

        # 4. Solve

        # compute optimal step
        # ATTENTION: This is an expensive step!!
        # ATTENTION: Use QR factorization and
        # column reordering for larger problems!!
        dX = - inv(J.transpose() @ J) @ J.transpose() @ r

        # update all poses
        for i in range(NUM_POSES):
            # we go very verbose here
            row = i * DoF
            size = DoF
            dx = dX[row:row+size]
            poses[i] = poses[i] + SE3Tangent(dx)

        # update all landmarks
        for k in range(NUM_LMKS):
            # we go very verbose here
            row = NUM_POSES * DoF + k * Dim
            size = Dim
            db = dX[row:row+size]
            landmarks[k] = landmarks[k] + db

        # DEBUG INFO
        print('residual norm: ', norm(r), ', step norm: ', norm(dX))

        # conditional exit
        if norm(dX) < 1e-6:
            break

    print('-----------------------------------------------')

    # Print results

    # solved problem
    print('posterior')
    for X in poses:
        print(
            'pose  : ',
            X.translation().transpose(),
            ' ', X.log().coeffs()[3:]
        )
    for b in landmarks:
        print('lmk : ', b.transpose())
    print('-----------------------------------------------')

    # ground truth
    print('ground truth')
    for X in poses_simu:
        print(
            'pose  : ',
            X.translation().transpose(), ' ',
            X.log().coeffs()[3:]
        )
    for b in landmarks_simu:
        print('lmk : ', b.transpose())
    print('-----------------------------------------------')
