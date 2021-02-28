/**
 * \file bundle_sam.cpp
 *
 *  Created on: Feb 28, 2021
 *      \author: pettni
 *
 * Adopted from se2_sam.cpp, see that file for explanations.
 */


// manif
#include "manif/Rn.h"
#include "manif/SE2.h"
#include "manif/Bundle.h"

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
using manif::SE2d;
using manif::SE2Tangentd;

static constexpr int DoF = SE2d::DoF;
static constexpr int Dim = SE2d::Dim;

// Define many data types (Tangent refers to the tangent of SE2)
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
static const int NUM_MEAS       = NUM_POSES * DoF + NUM_FACTORS * Dim;
static const int MAX_ITER       = 20;           // for the solver


// bundle state type to optimize over
using BundleT = manif::Bundle<double,
    manif::SE2,
    manif::SE2,
    manif::SE2,
    manif::R2,
    manif::R2,
    manif::R2,
    manif::R2,
    manif::R2
>;

// Insert a relative pose factor from pose XI to pose XJ
// into the residual-jacobian pair (r, J)
template<std::size_t XI, std::size_t XJ>
void add_pose_factor(
    const BundleT & X,
    const SE2Tangentd & control,
    const MatrixT & W,
    Eigen::Ref<Eigen::Matrix<double, 3, 1>> r,
    Eigen::Ref<Eigen::Matrix<double, 3, BundleT::DoF>> J)
{
    // index start position and length in the DoF of BundleT
    constexpr int BegI = manif::internal::intseq_element<XI, BundleT::BegDoF>::value;
    constexpr int LenI = manif::internal::intseq_element<XI, BundleT::LenDoF>::value;
    constexpr int BegJ = manif::internal::intseq_element<XJ, BundleT::BegDoF>::value;
    constexpr int LenJ = manif::internal::intseq_element<XJ, BundleT::LenDoF>::value;

    MatrixT         J_d_xi, J_d_xj; // Jacobian of motion wrt poses i and j

    auto d = X.element<XJ>().rminus(X.element<XI>(), J_d_xj, J_d_xi);
    r = W * (d - control).coeffs();
    J.setZero();
    J.block<3, LenI>(0, BegI) = W * J_d_xi;
    J.block<3, LenJ>(0, BegJ) = W * J_d_xj;
}


// Insert a landmark measurement factor of landmark LK
// from pose XI into the residual-jacobian pair (r, J)
template<std::size_t XI, std::size_t LK>
void add_beacon_factor(
    const BundleT & X,
    const VectorY & measurement,
    const MatrixY & S,
    Eigen::Ref<Eigen::Matrix<double, 2, 1>> r,
    Eigen::Ref<Eigen::Matrix<double, 2, BundleT::DoF>> J)
{
    // index start position and length in the DoF of BundleT
    constexpr int BegX = manif::internal::intseq_element<XI, BundleT::BegDoF>::value;
    constexpr int LenX = manif::internal::intseq_element<XI, BundleT::LenDoF>::value;
    constexpr int BegLMK = manif::internal::intseq_element<NUM_POSES + LK, BundleT::BegDoF>::value;
    constexpr int LenLMK = manif::internal::intseq_element<NUM_POSES + LK, BundleT::LenDoF>::value;

    MatrixT         J_ix_x;         // Jacobian of inverse pose wrt pose
    MatrixYT        J_e_ix;         // Jacobian of measurement expectation wrt inverse pose
    MatrixYT        J_e_x;          // Jacobian of measurement expectation wrt pose
    MatrixYB        J_e_b;          // Jacobian of measurement expectation wrt lmk

    auto e = X.element<XI>().inverse(J_ix_x).act(X.element<NUM_POSES + LK>().coeffs(), J_e_ix, J_e_b);
    J_e_x = J_e_ix * J_ix_x;
    r = S * (e - measurement);
    J.setZero();
    J.block<Dim, LenX>(0, BegX) = S * J_e_x;
    J.block<Dim, LenLMK>(0, BegLMK) = S * J_e_b;
}


int main()
{
    std::srand((unsigned int) time(0));

    // DEBUG INFO
    cout << endl;
    cout << "2D Smoothing and Mapping. 3 poses, 5 landmarks." << endl;
    cout << "-----------------------------------------------" << endl;
    cout << std::fixed   << std::setprecision(3) << std::showpos;

    // START CONFIGURATION
    //
    //

    // Define the robot pose elements
    SE2d         X_simu,    // pose of the simulated robot
                 Xi,        // robot pose at time i
                 Xj;        // robot pose at time j
    vector<SE2d> poses,     // estimator poses
                 poses_simu;// simulator poses
    Xi.setIdentity();
    X_simu.setIdentity();


    // Define a control vector and its noise and covariance in the tangent of SE2
    SE2Tangentd         u;          // control signal, generic
    SE2Tangentd         u_nom;      // nominal control signal
    ArrayT              u_sigmas;   // control noise std specification
    VectorT             u_noise;    // control noise
    MatrixT             Q;          // Covariance
    MatrixT             W;          // sqrt Info
    vector<SE2Tangentd> controls;   // robot controls

    u_nom     << 0.1, 0.0, 0.05;
    u_sigmas  << 0.01, 0.01, 0.01;
    Q         = (u_sigmas * u_sigmas).matrix().asDiagonal();
    W         =  u_sigmas.inverse()  .matrix().asDiagonal(); // this is Q^(-T/2)

    // Landmarks in R^2 and map
    VectorB b; // Landmark, generic
    vector<VectorB> landmarks(NUM_LMKS), landmarks_simu;
    {
        // Define five landmarks (beacons) in R^2
        VectorB b0, b1, b2, b3, b4;
        b0 << 3.0,  0.0;
        b1 << 2.0, -1.0;
        b2 << 2.0,  1.0;
        b3 << 3.0, -1.0;
        b4 << 3.0,  1.0;
        landmarks_simu.push_back(b0);
        landmarks_simu.push_back(b1);
        landmarks_simu.push_back(b2);
        landmarks_simu.push_back(b3);
        landmarks_simu.push_back(b4);
    } // destroy b0...b4

    // Define the beacon's measurements in R^2
    VectorY             y, y_noise;
    ArrayY              y_sigmas;
    MatrixY             R; // Covariance
    MatrixY             S; // sqrt Info
    vector<map<int,VectorY>> measurements(NUM_POSES); // y = measurements[pose_id][lmk_id]

    y_sigmas << 0.001, 0.001;
    R        = (y_sigmas * y_sigmas).matrix().asDiagonal();
    S        =  y_sigmas.inverse()  .matrix().asDiagonal(); // this is R^(-T/2)

    // Problem-size variables

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
    poses.      push_back(Xi + SE2Tangentd::Random());  // use very noisy priors

    // temporal loop
    for (int i = 0; i < NUM_POSES; ++i)
    {
        // make measurements
        for (const auto& k : pairs[i])
        {
            // simulate measurement
            b       = landmarks_simu[k];              // lmk coordinates in world frame
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
            poses.      push_back(Xi + SE2Tangentd::Random()); // use very noisy priors
            controls.   push_back(u_nom + u_noise);
        }
    }

    //// Estimator #################################################################

    // Insert priors into bundle state
    BundleT X(poses[0], poses[1], poses[2],
        manif::R2d(landmarks[0]), manif::R2d(landmarks[1]),
        manif::R2d(landmarks[2]), manif::R2d(landmarks[3]),
        manif::R2d(landmarks[4]));

    cout << "prior" << std::showpos << endl;
    cout << "pose  :" << X.element<0>().translation().transpose() << " " << X.element<0>().angle() << endl;
    cout << "pose  :" << X.element<1>().translation().transpose() << " " << X.element<1>().angle() << endl;
    cout << "pose  :" << X.element<2>().translation().transpose() << " " << X.element<2>().angle() << endl;

    cout << "lmk   :" << X.element<3>() << endl;
    cout << "lmk   :" << X.element<4>() << endl;
    cout << "lmk   :" << X.element<5>() << endl;
    cout << "lmk   :" << X.element<6>() << endl;
    cout << "lmk   :" << X.element<7>() << endl;
    cout << "-----------------------------------------------" << endl;

    // iterate
    // DEBUG INFO
    cout << "iterations" << std::noshowpos << endl;
    for (int iteration = 0; iteration < MAX_ITER; ++iteration)
    {
        Matrix<double, NUM_MEAS, BundleT::DoF>    J;  // full Jacobian
        Matrix<double, NUM_MEAS, 1>               r;  // full residual

        int row = 0;  // keep track of row in J and r

        // first residual: prior
        r.segment<DoF>(row) = X.element<0>().lminus(SE2d::Identity(), J.block<DoF, DoF>(row, 0)).coeffs();
        row += DoF;

        // motion residuals
        add_pose_factor<0, 1>(X, controls[0], W, r.segment<DoF>(row), J.block<DoF, BundleT::DoF>(row, 0));
        row += DoF;

        add_pose_factor<1, 2>(X, controls[1], W, r.segment<DoF>(row), J.block<DoF, BundleT::DoF>(row, 0));
        row += DoF;

        // measurement residuals
        add_beacon_factor<0, 0>(X, measurements[0][0], S, r.segment<Dim>(row), J.block<Dim, BundleT::DoF>(row, 0));
        row += Dim;
        add_beacon_factor<0, 1>(X, measurements[0][1], S, r.segment<Dim>(row), J.block<Dim, BundleT::DoF>(row, 0));
        row += Dim;
        add_beacon_factor<0, 3>(X, measurements[0][3], S, r.segment<Dim>(row), J.block<Dim, BundleT::DoF>(row, 0));
        row += Dim;

        add_beacon_factor<1, 0>(X, measurements[1][0], S, r.segment<Dim>(row), J.block<Dim, BundleT::DoF>(row, 0));
        row += Dim;
        add_beacon_factor<1, 2>(X, measurements[1][2], S, r.segment<Dim>(row), J.block<Dim, BundleT::DoF>(row, 0));
        row += Dim;
        add_beacon_factor<1, 4>(X, measurements[1][4], S, r.segment<Dim>(row), J.block<Dim, BundleT::DoF>(row, 0));
        row += Dim;

        add_beacon_factor<2, 1>(X, measurements[2][1], S, r.segment<Dim>(row), J.block<Dim, BundleT::DoF>(row, 0));
        row += Dim;
        add_beacon_factor<2, 2>(X, measurements[2][2], S, r.segment<Dim>(row), J.block<Dim, BundleT::DoF>(row, 0));
        row += Dim;
        add_beacon_factor<2, 4>(X, measurements[2][4], S, r.segment<Dim>(row), J.block<Dim, BundleT::DoF>(row, 0));
        row += Dim;

        // 4. Solve -----------------------------------------------------------------

        // compute optimal step
        // ATTENTION: This is an expensive step!!
        // ATTENTION: Use QR factorization and column reordering for larger problems!!
        const auto dX = (-(J.transpose() * J).inverse() * J.transpose() * r).eval();

        // update estimate
        X += BundleT::Tangent(dX);

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
    cout << "pose  :" << X.element<0>().translation().transpose() << " " << X.element<0>().angle() << endl;
    cout << "pose  :" << X.element<1>().translation().transpose() << " " << X.element<1>().angle() << endl;
    cout << "pose  :" << X.element<2>().translation().transpose() << " " << X.element<2>().angle() << endl;

    cout << "lmk   :" << X.element<3>() << endl;
    cout << "lmk   :" << X.element<4>() << endl;
    cout << "lmk   :" << X.element<5>() << endl;
    cout << "lmk   :" << X.element<6>() << endl;
    cout << "lmk   :" << X.element<7>() << endl;

    cout << "-----------------------------------------------" << endl;

    // ground truth
    cout << "ground truth" << std::showpos << endl;
    for (const auto& X : poses_simu)
        cout << "pose  : " << X.translation().transpose() << " " << X.angle() << endl;
    for (const auto& b : landmarks_simu)
        cout << "lmk : " << b.transpose() << endl;
    cout << "-----------------------------------------------" << endl;

    return 0;
}
