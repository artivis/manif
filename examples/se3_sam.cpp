/**
 * \file se3_sam.cpp
 *
 *  Created on: Dec 13, 2018
 *      \author: jsola
 *
 *  ------------------------------------------------------------
 *  This file is:
 *  (c) 2018 Joan Sola @ IRI-CSIC, Barcelona, Catalonia
 *
 *  This file is part of `manif`, a C++ template-only library
 *  for Lie theory targeted at estimation for robotics.
 *  Manif is:
 *  (c) 2018 Jeremie Deray @ IRI-UPC, Barcelona
 *  ------------------------------------------------------------
 *
 *  ------------------------------------------------------------
 *  Demonstration example:
 *
 *  3D smoothing and mapping (SAM).
 *
 *  See se2_sam.cpp          for a 2D version of this example.
 *  See se3_localization.cpp for a simpler localization example using EKF.
 *  ------------------------------------------------------------
 *
 *  This demo corresponds to the 3D version of the application
 *  in chapter V, section B, in the paper Sola-18,
 *  [https://arxiv.org/abs/1812.01537].
 *
 *  The following is an abstract of the content of the paper.
 *  Please consult the paper for better reference.
 *
 *
 *  We consider a robot in 3D space surrounded by a small
 *  number of punctual landmarks or _beacons_.
 *  The robot receives control actions in the form of axial
 *  and angular velocities, and is able to measure the location
 *  of the beacons w.r.t its own reference frame.
 *
 *  The robot pose X_i is in SE(3) and the beacon positions b_k in R^3,
 *
 *      X_i = |  R_i   t_i |        // position and orientation
 *            |   0     1  |
 *
 *      b_k = (bx_k, by_k, bz_k)    // lmk coordinates in world frame
 *
 *  The control signal u is a twist in se(3) comprising longitudinal
 *  velocity vx and angular velocity wz, with no other velocity
 *  components, integrated over the sampling time dt.
 *
 *      u = (vx*dt, 0, 0, 0, 0, w*dt)
 *
 *  The control is corrupted by additive Gaussian noise u_noise,
 *  with covariance
 *
 *      Q = diagonal(sigma_x^2, sigma_y^2, sigma_z^2, sigma_roll^2, sigma_pitch^2, sigma_yaw^2).
 *
 *  This noise accounts for possible lateral and rotational slippage
 *  through non-zero values of sigma_y, sigma_z, sigma_roll and sigma_pitch.
 *
 *  At the arrival of a control u, a new robot pose is created at
 *
 *      X_j = X_i * Exp(u) = X_i + u.
 *
 *  This new pose is then added to the graph.
 *
 *  Landmark measurements are of the range and bearing type,
 *  though they are put in Cartesian form for simplicity,
 *
 *      y = (yx, yy, yz)        // lmk coordinates in robot frame
 *
 *  Their noise n is zero mean Gaussian, and is specified
 *  with a covariances matrix R.
 *  We notice the rigid motion action y_ik = h(X_i,b_k) = X_i^-1 * b_k
 *  (see appendix D).
 *
 *
 *  The world comprises 5 landmarks.
 *  Not all of them are observed from each pose.
 *  A set of pairs pose--landmark is created to establish which
 *  landmarks are observed from each pose.
 *  These pairs can be observed in the factor graph, as follows.
 *
 *  The factor graph of the SAM problem looks like this:
 *
 *                  ------- b1
 *          b3    /         |
 *          |   /       b4  |
 *          | /       /    \|
 *          X0 ---- X1 ---- X2
 *          | \   /   \   /
 *          |   b0      b2
 *          *
 *
 *  where:
 *    - X_i are SE3 robot poses
 *    - b_k are R^3 landmarks or beacons
 *    - * is a pose prior to anchor the map and make the problem observable
 *    - segments indicate measurement factors:
 *      - motion measurements from X_i to X_j
 *      - landmark measurements from X_i to b_k
 *      - absolute pose measurement from X0 to * (the origin of coordinates)
 *
 *  We thus declare 9 factors pose---landmark, as follows:
 *
 *    poses ---  lmks
 *      x0  ---  b0
 *      x0  ---  b1
 *      x0  ---  b3
 *      x1  ---  b0
 *      x1  ---  b2
 *      x1  ---  b4
 *      x2  ---  b1
 *      x2  ---  b2
 *      x2  ---  b4
 *
 *
 *  The main variables are summarized again as follows
 *
 *      Xi  : robot pose at time i, SE(3)
 *      u   : robot control, (v*dt; 0; 0; 0; 0; w*dt) in se(3)
 *      Q   : control perturbation covariance
 *      b   : landmark position, R^3
 *      y   : Cartesian landmark measurement in robot frame, R^3
 *      R   : covariance of the measurement noise
 *
 *
 *  We define the state to estimate as a manifold composite:
 *
 *      X in  < SE3, SE3, SE3, R^3, R^3, R^3, R^3, R^3 >
 *
 *      X  =  <  X0,  X1,  X2,  b0,  b1,  b2,  b3,  b4 >
 *
 *  The estimation error dX is expressed
 *  in the tangent space at X,
 *
 *      dX in  < se3, se3, se3, R^3, R^3, R^3, R^3, R^3 >
 *          ~  < R^6, R^6, R^6, R^3, R^3, R^3, R^3, R^3 > = R^33
 *
 *      dX  =  [ dx0, dx1, dx2, db0, db1, db2, db3, db4 ] in R^33
 *
 *  with
 *      dx_i: pose error in se(3) ~ R^6
 *      db_k: landmark error in R^3
 *
 *
 *  The prior, motion and measurement models are
 *
 *    - for the prior factor:
 *        p_0     = X_0
 *
 *    - for the motion factors:
 *        d_ij    = X_j (-) X_i = log(X_i.inv * X_j)  // motion expectation equation
 *
 *    - for the measurement factors:
 *        e_ik    = h(X_i, b_k) = X_i^-1 * b_k        // measurement expectation equation
 *
 *
 *
 *  The algorithm below comprises first a simulator to
 *  produce measurements, then uses these measurements
 *  to estimate the state, using a graph representation
 *  and Lie-based non-linear iterative least squares solver
 *  that uses the pseudo-inverse method.
 *
 *  This file has plain code with only one main() function.
 *  There are no function calls other than those involving `manif`.
 *
 *  Printing the prior state (before solving) and posterior state (after solving),
 *  together with a ground-truth state defined by the simulator
 *  allows for evaluating the quality of the estimates.
 *
 *  This information is complemented with the evolution of
 *  the optimizer's residual and optimal step norms. This allows
 *  for evaluating the convergence of the optimizer.
 */



// manif
#include "manif/SE3.h"

// Std
#include <vector>
#include <map>
#include <list>
#include <cstdlib>

// Debug
#include <iostream>
#include <iomanip>

// std shortcuts and namespaces
using std::cout;
using std::endl;
using std::vector;
using std::map;
using std::list;
using std::pair;

// Eigen namespace
using namespace Eigen;

// manif namespace and shortcuts
using manif::SE3d;
using manif::SE3Tangentd;

static constexpr int DoF = SE3d::DoF;
static constexpr int Dim = SE3d::Dim;

// Define many data types (Tangent refers to the tangent of SE3)
typedef Array<double,  DoF, 1>      ArrayT;     // tangent-size array
typedef Matrix<double, DoF, 1>      VectorT;    // tangent-size vector
typedef Matrix<double, DoF, DoF>    MatrixT;    // tangent-size square matrix
typedef Matrix<double, Dim, 1>      VectorB;    // landmark-size vector
typedef Array<double,  Dim, 1>      ArrayY;     // measurement-size array
typedef Matrix<double, Dim, 1>      VectorY;    // measurement-size vector
typedef Matrix<double, Dim, Dim>    MatrixY;    // measurement-size square matrix
typedef Matrix<double, Dim, DoF>    MatrixYT;   // measurement x tangent size matrix
typedef Matrix<double, Dim, Dim>    MatrixYB;   // measurement x landmark size matrix

// some experiment constants
static const int NUM_POSES      = 3;
static const int NUM_LMKS       = 5;
static const int NUM_FACTORS    = 9;
static const int NUM_STATES     = NUM_POSES * DoF + NUM_LMKS    * Dim;
static const int NUM_MEAS       = NUM_POSES * DoF + NUM_FACTORS * Dim;
static const int MAX_ITER       = 20;           // for the solver

int main()
{
    std::srand((unsigned int) time(0));

    // DEBUG INFO
    cout << endl;
    cout << "3D Smoothing and Mapping. 3 poses, 5 landmarks." << endl;
    cout << "-----------------------------------------------" << endl;
    cout << std::fixed   << std::setprecision(3) << std::showpos;

    // START CONFIGURATION
    //
    //

    // Define the robot pose elements
    SE3d         X_simu,    // pose of the simulated robot
                 Xi,        // robot pose at time i
                 Xj;        // robot pose at time j
    vector<SE3d> poses,     // estimator poses
                 poses_simu;// simulator poses
    Xi.setIdentity();
    X_simu.setIdentity();


    // Define a control vector and its noise and covariance in the tangent of SE3
    SE3Tangentd         u;          // control signal, generic
    SE3Tangentd         u_nom;      // nominal control signal
    ArrayT              u_sigmas;   // control noise std specification
    VectorT             u_noise;    // control noise
    MatrixT             Q;          // Covariance
    MatrixT             W;          // sqrt Info
    vector<SE3Tangentd> controls;   // robot controls

    u_nom    << 0.1, 0.0, 0.0, 0.0, 0.0, 0.05;
    u_sigmas << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    Q        = (u_sigmas * u_sigmas).matrix().asDiagonal();
    W        =  u_sigmas.inverse()  .matrix().asDiagonal(); // this is Q^(-T/2)

    // Landmarks in R^3 and map
    VectorB b; // Landmark, generic
    vector<VectorB> landmarks(NUM_LMKS), landmarks_simu;
    {
        // Define five landmarks (beacons) in R^3
        VectorB b0, b1, b2, b3, b4;
        b0 << 3.0,  0.0,  0.0;
        b1 << 2.0, -1.0, -1.0;
        b2 << 2.0, -1.0,  1.0;
        b3 << 2.0,  1.0,  1.0;
        b4 << 2.0,  1.0, -1.0;
        landmarks_simu.push_back(b0);
        landmarks_simu.push_back(b1);
        landmarks_simu.push_back(b2);
        landmarks_simu.push_back(b3);
        landmarks_simu.push_back(b4);
    } // destroy b0...b4

    // Define the beacon's measurements in R^3
    VectorY             y, y_noise;
    ArrayY              y_sigmas;
    MatrixY             R; // Covariance
    MatrixY             S; // sqrt Info
    vector<map<int,VectorY>>    measurements(NUM_POSES); // y = measurements[pose_id][lmk_id]

    y_sigmas << 0.001, 0.001, 0.001; y_sigmas /= 1.0;
    R        = (y_sigmas * y_sigmas).matrix().asDiagonal();
    S        =  y_sigmas.inverse()  .matrix().asDiagonal(); // this is R^(-T/2)

    // Declare some temporaries
    SE3Tangentd     d;              // motion expectation d = Xj (-) Xi = Xj.minus ( Xi )
    VectorY         e;              // measurement expectation e = h(X, b)
    MatrixT         J_d_xi, J_d_xj; // Jacobian of motion wrt poses i and j
    MatrixT         J_ix_x;         // Jacobian of inverse pose wrt pose
    MatrixYT        J_e_ix;         // Jacobian of measurement expectation wrt inverse pose
    MatrixYT        J_e_x;          // Jacobian of measurement expectation wrt pose
    MatrixYB        J_e_b;          // Jacobian of measurement expectation wrt lmk
    SE3Tangentd     dx;             // optimal pose correction step
    VectorB         db;             // optimal landmark correction step

    // Problem-size variables
    Matrix<double, NUM_STATES, 1>           dX; // optimal update step for all the SAM problem
    Matrix<double, NUM_MEAS, NUM_STATES>    J;  // full Jacobian
    Matrix<double, NUM_MEAS, 1>             r;  // full residual

    /*
     * The factor graph of the SAM problem looks like this:
     *
     *                  ------- b1
     *          b3    /         |
     *          |   /       b4  |
     *          | /       /    \|
     *          X0 ---- X1 ---- X2
     *          | \   /   \   /
     *          |   b0      b2
     *          *
     *
     * where:
     *   - Xi are poses
     *   - bk are landmarks or beacons
     *   - * is a pose prior to anchor the map and make the problem observable
     *
     * Define pairs of nodes for all the landmark measurements
     * There are 3 pose nodes [0..2] and 5 landmarks [0..4].
     * A pair pose -- lmk means that the lmk was measured from the pose
     * Each pair declares a factor in the factor graph
     * We declare 9 pairs, or 9 factors, as follows:
     */
    vector<list<int>> pairs(NUM_POSES);
    pairs[0].push_back(0);  // 0-0
    pairs[0].push_back(1);  // 0-1
    pairs[0].push_back(3);  // 0-3
    pairs[1].push_back(0);  // 1-0
    pairs[1].push_back(2);  // 1-2
    pairs[1].push_back(4);  // 1-4
    pairs[2].push_back(1);  // 2-1
    pairs[2].push_back(2);  // 2-2
    pairs[2].push_back(4);  // 2-4

    //
    //
    // END CONFIGURATION


    //// Simulator ###################################################################
    poses_simu. push_back(X_simu);
    poses.      push_back(Xi + (SE3Tangentd::Random()*0.1));  // use very noisy priors

    // temporal loop
    for (int i = 0; i < NUM_POSES; ++i)
    {
        // make measurements
        for (const auto& k : pairs[i])
        {
            // simulate measurement
            b       = landmarks_simu[k];                // lmk coordinates in world frame
            y_noise = y_sigmas * ArrayY::Random();      // measurement noise
            y       = X_simu.inverse().act(b);          // landmark measurement, before adding noise

            // add noise and compute prior lmk from prior pose
            measurements[i][k]  = y + y_noise;           // store noisy measurements
            b                   = Xi.act(y + y_noise);   // mapped landmark with noise
            landmarks[k]        = b + VectorB::Random(); // use very noisy priors
        }

        // make motions
        if (i < NUM_POSES - 1) // do not make the last motion since we're done after 3rd pose
        {
            // move simulator, without noise
            X_simu = X_simu + u_nom;

            // move prior, with noise
            u_noise = u_sigmas * ArrayT::Random();
            Xi = Xi + (u_nom + u_noise);

            // store
            poses_simu. push_back(X_simu);
            poses.      push_back(Xi + (SE3Tangentd::Random()*0.1)); // use very noisy priors
            controls.   push_back(u_nom + u_noise);
        }
    }

    //// Estimator #################################################################

    // DEBUG INFO
    cout << "prior" << std::showpos << endl;
    for (const auto& X : poses)
        cout << "pose  : " << X.translation().transpose() << " " << X.asSO3().log() << endl;
    for (const auto& b : landmarks)
        cout << "lmk : " << b.transpose() << endl;
    cout << "-----------------------------------------------" << endl;


    // iterate
    // DEBUG INFO
    cout << "iterations" << std::noshowpos << endl;
    for (int iteration = 0; iteration < MAX_ITER; ++iteration)
    {
        // Clear residual vector and Jacobian matrix
        r .setZero();
        J .setZero();

        // row and column for the full Jacobian matrix J, and row for residual r
        int row = 0, col = 0;

        // 1. evaluate prior factor ---------------------
        /*
         *  NOTE (see Chapter 2, Section E, of Sola-18):
         *
         *  To compute any residual, we consider the following variables:
         *      r: residual
         *      e: expectation
         *      y: prior specification 'measurement'
         *      W: sqrt information matrix of the measurement noise.
         *
         *  In case of a non-trivial prior measurement, we need to consider
         *  the nature of it: is it a global or a local specification?
         *
         *  When prior information `y` is provided in the global reference,
         *  we need a left-minus operation (.-) to compute the residual.
         *  This is usually the case for pose priors, since it is natural
         *  to specify position and orientation wrt a global reference,
         *
         *     r = W * (e (.-) y)
         *       = W * (e * y.inv).log()
         *
         *  When `y` is provided as a local reference, then right-minus (-.) is required,
         *
         *     r = W * (e (-.) y)
         *       = W * (y.inv * e).log()
         *
         *  Notice that if y = Identity() then local and global residuals are the same.
         *
         *
         *  Here, expectation, measurement and info matrix are trivial, as follows
         *
         *  expectation
         *     e = poses[0];            // first pose
         *
         *  measurement
         *     y = SE3d::Identity()     // robot at the origin
         *
         *  info matrix:
         *     W = I                    // trivial
         *
         *  residual
         *     r = W * (poses[0] (.-) measurement) = log(poses[0] * Id.inv) = poses[0].log()
         *
         *  Jacobian matrix :
         *     J_r_p0 = Jr_inv(log(poses[0]))         // see proof below
         *
         *     Proof: Let p0 = poses[0] and y = measurement. We have the partials
         *       J_r_p0 = W^(T/2) * d(log(p0 * y.inv)/d(poses[0])
         *
         *     with W = i and y = I. Since d(log(r))/d(r) = Jr_inv(r) for any r in the Lie algebra, we have
         *       J_r_p0 = Jr_inv(log(p0))
         */

        // residual and Jacobian.
        // Notes:
        //   We have residual = expectation - measurement, in global tangent space
        //   We have the Jacobian in J_r_p0 = J.block<DoF, DoF>(row, col);
        // We compute the whole in a one-liner:
        r.segment<DoF>(row)         = poses[0].lminus(SE3d::Identity(), J.block<DoF, DoF>(row, col)).coeffs();

        // advance rows
        row += DoF;

        // loop poses
        for (int i = 0; i < NUM_POSES; ++i)
        {
            // 2. evaluate motion factors -----------------------
            if (i < NUM_POSES - 1) // do not make the last motion since we're done after 3rd pose
            {
                int j = i + 1; // this is next pose's id

                // recover related states and data
                Xi = poses[i];
                Xj = poses[j];
                u  = controls[i];

                // expectation (use right-minus since motion measurements are local)
                d  = Xj.rminus(Xi, J_d_xj, J_d_xi); // expected motion = Xj (-) Xi

                // residual
                r.segment<DoF>(row)         = W * (d - u).coeffs(); // residual

                // Jacobian of residual wrt first pose
                col = i * DoF;
                J.block<DoF, DoF>(row, col) = W * J_d_xi;

                // Jacobian of residual wrt second pose
                col = j * DoF;
                J.block<DoF, DoF>(row, col) = W * J_d_xj;

                // advance rows
                row += DoF;
            }

            // 3. evaluate measurement factors ---------------------------
            for (const auto& k : pairs[i])
            {
                // recover related states and data
                Xi = poses[i];
                b  = landmarks[k];
                y  = measurements[i][k];

                // expectation
                e       = Xi.inverse(J_ix_x).act(b, J_e_ix, J_e_b); // expected measurement = Xi.inv * bj
                J_e_x   = J_e_ix * J_ix_x;                          // chain rule

                // residual
                r.segment<Dim>(row)         = S * (e - y);

                // Jacobian of residual wrt pose
                col = i * DoF;
                J.block<Dim, DoF>(row, col) = S * J_e_x;

                // Jacobian of residual wrt lmk
                col = NUM_POSES * DoF + k * Dim;
                J.block<Dim, Dim>(row, col) = S * J_e_b;

                // advance rows
                row += Dim;
            }
        }

        // 4. Solve -----------------------------------------------------------------

        // compute optimal step
        // ATTENTION: This is an expensive step!!
        // ATTENTION: Use QR factorization and column reordering for larger problems!!
        dX = - (J.transpose() * J).inverse() * J.transpose() * r;

        // update all poses
        for (int i = 0; i < NUM_POSES; ++i)
        {
            // we go very verbose here
            int row             = i * DoF;
            constexpr int size  = DoF;
            dx                  = dX.segment<size>(row);
            poses[i]            = poses[i] + dx;
        }

        // update all landmarks
        for (int k = 0; k < NUM_LMKS; ++k)
        {
            // we go very verbose here
            int row             = NUM_POSES * DoF + k * Dim;
            constexpr int size  = Dim;
            db                  = dX.segment<size>(row);
            landmarks[k]        = landmarks[k] + db;
        }


        // DEBUG INFO
        cout << "residual norm: " << std::scientific << r.norm() << ", step norm: " << dX.norm() << endl;

        // conditional exit
        if (dX.norm() < 1e-6) break;
    }
    cout << "-----------------------------------------------" << endl;


    //// Print results ####################################################################

    cout << std::fixed;

    // solved problem
    cout << "posterior" << std::showpos << endl;
    for (const auto& X : poses)
        cout << "pose  : " << X.translation().transpose() << " " << X.asSO3().log() << endl;
    for (const auto& b : landmarks)
        cout << "lmk : " << b.transpose() << endl;
    cout << "-----------------------------------------------" << endl;

    // ground truth
    cout << "ground truth" << std::showpos << endl;
    for (const auto& X : poses_simu)
        cout << "pose  : " << X.translation().transpose() << " " << X.asSO3().log() << endl;
    for (const auto& b : landmarks_simu)
        cout << "lmk : " << b.transpose() << endl;
    cout << "-----------------------------------------------" << endl;

    return 0;
}
