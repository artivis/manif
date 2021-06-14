^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package manif
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2021-06-14)
------------------
* Merge pull request `#231 <https://github.com/artivis/manif/issues/231>`_ from artivis/devel
  [sync] Prepare release 0.0.4
* Add the possibility to use a system wide installation of tl-optional (`#207 <https://github.com/artivis/manif/issues/207>`_)
  * Fix typo in tl-optional folder name
  * Add the possibility to use a system-wide installation of tl-optional
  * Bump tl-optional to the latest version (1.0.x)
  * Update CMakeLists.txt
* Merge pull request `#227 <https://github.com/artivis/manif/issues/227>`_ from artivis/feature/benchmarking
  Add benchmarking & some opti
* list some new projects using manif
* run tests in debug & release on ubuntu
* fix test avg
* rm unnecessary cmake warning suppression
* fix some warnings
* add MANIF_UNUSED_VARIABLE
* add se_2_3 rjacinv/ljacinv
* add se2 rjacinv/ljacinv
* SO3TangentBase rm temp var
* se_2_3 adj no alias
* se_2_3 act jac no alias
* se_2_3 isometry small opti
* se_2_3 tan avoid temp var
* so3 act noalias
* so3 compose avoid temp var
* so3 log jac small opti
* so3 exp jac small opti
* so2 act noalias
* se3 adj small opti
* lipstick
* review lminus jac logic
* add benchmarking
* Merge pull request `#224 <https://github.com/artivis/manif/issues/224>`_ from artivis/fix/tests_at_around_I
  Run tests at/around identity
* fix SO3 ljac/ljacinv small angle
* remove eps_s
* run tests at & around identity
* factorize some test-related code
* Merge pull request `#222 <https://github.com/artivis/manif/issues/222>`_ from artivis/fix/se_2_3_tan_coeffs
  Fix function call
* small fix
* add video link
* Merge pull request `#219 <https://github.com/artivis/manif/issues/219>`_ from artivis/feature/install_gtest_helpers
  Install gtest helpers
* rename & install gtest helpers
* Merge pull request `#214 <https://github.com/artivis/manif/issues/214>`_ from artivis/fix/project_version
  Set project version
* Merge pull request `#217 <https://github.com/artivis/manif/issues/217>`_ from artivis/fix/float_support
  Fixes for float support
* ignore build* folders
* test SO3 for float
* relax tests for float
* specialize Constants for float
* cast randPointInBall to appropriate scalar type
* Use Eigen::Quaternion::unitRandom where possible
* bump CMake version & set project version
* Merge pull request `#211 <https://github.com/artivis/manif/issues/211>`_ from GiulioRomualdi/pybind11
  [manifpy] General improvements in SO3 and SE3 objects
* Update python bindings so3 tests
* Update python bindings se3 tests
* Implement quat() methods in SO3 python bindings
* Avoid to use Eigen::Vector4d in bindings_so3.cpp
* Apply changes in bindings_se3.cpp after the review
* Implement TEST_SO3_SET_QUATERNION
* Implement SO3Base::quat() methods
* Update test_so3.py accordingly to 69d4c42547e34fdf17b4eef7fc1d2a0f88736e08
* Update test_se3.py accordingly to 3bd2cd2adb8e485df07aab522e595e9959b902ec and 0e07ab58a4854657ec7566cc0a5121b088629aae
* Add translation and quat properties in SE3 python bindings
* Add the possibility to construct a SE3 from two vectors in python bindings
* Add the possibility to construct a SO3 from a vector in python bindings
* Merge pull request `#213 <https://github.com/artivis/manif/issues/213>`_ from artivis/fix/cppcheck_1.90
  Fix/cppcheck 1.90
* disable some warnings/errors picked up by cppcheck 1.90
* fix some warnings picked up by cppcheck 1.90
* Add GitHub Actions workflow for Visual Studio (`#206 <https://github.com/artivis/manif/issues/206>`_)
  * Add GitHub Actions workflow for Visual Studio
* Fix assert hint
  closes `#204 <https://github.com/artivis/manif/issues/204>`_
* use latest doxygen release in doc CI
* use latest doxygen release in doc CI
* Fix doc deploy
* Misc small doc fixes (`#202 <https://github.com/artivis/manif/issues/202>`_)
  * fix python doc build
  * fix links in readme
  * cleanup comments
  * fix latex rendering - no idea why tho
* Merge pull request `#201 <https://github.com/artivis/manif/issues/201>`_ from artivis/feature/python
  Add Python bindings
* new doc site to include Python
* add Python CI job
* add Python examples
* add Python tests
* add Python wrappers
* Merge pull request `#195 <https://github.com/artivis/manif/issues/195>`_ from artivis/fix/ceres_tests
  Adding Ceres::Jet unit tests
* Merge pull request `#200 <https://github.com/artivis/manif/issues/200>`_ from artivis/fix/misc
  Miscellaneous small fixes
* Add SE2 UKF-M example (`#176 <https://github.com/artivis/manif/issues/176>`_)
  * add se2 ukfm example
  * add demo to README
  Co-authored-by: Joan Solà <jsola@iri.upc.edu>
* Disambiguate v/w/a -> lin/ang/lin2 (`#190 <https://github.com/artivis/manif/issues/190>`_)
  * disambiguate v/w/a -> lin/ang/lin2
* doc fix
* add jac tests rplus/lplus/act
* add jac test log/compose/between
* add missing files
* wip Ceres::Jet unit tests
* specialize traitscast for Rn
* cosmetic
* delete .travis
* tmp disable macos-11 CI
* examples: init rand and reduce noise
* fix optional ret val
* fix tangent constexpr decl
* fix group ** constness
* Merge pull request `#189 <https://github.com/artivis/manif/issues/189>`_ from artivis/fix/inner_weight
  Disambiguate w -> innerWeights
* disambiguate w -> innerWeights
* Contributors: Giulio Romualdi, Jeremie Deray, Silvio Traversaro, artivis

0.0.3 (2021-01-17)
------------------
* Add traits `is_ad` (`#199 <https://github.com/artivis/manif/issues/199>`_)
  * add traits is_ad & ceres spe
* Merge pull request `#194 <https://github.com/artivis/manif/issues/194>`_ from pettni/minor_fixes
  Minor fixes: use known inverses, fix ambiguous ternary types
* Merge pull request `#188 <https://github.com/artivis/manif/issues/188>`_ from artivis/fix/liegroup_act
  Fix LieGroupBase::act signature
* Fix ambiguous ternary type
* Use explicit inverses when known
* fix LieGroupBase::act signature
* Merge pull request `#183 <https://github.com/artivis/manif/issues/183>`_ from artivis/fix/181
  fix compilation in release
* Merge pull request `#186 <https://github.com/artivis/manif/issues/186>`_ from artivis/fix/const_data_ptr
  Fix missing return in const data()
* Fix SO3/SE3Tangent random (`#182 <https://github.com/artivis/manif/issues/182>`_)
  * fix SO3/SE3Tangent random
  * fix exp.log test
* fix compilation in release
  closes `#181 <https://github.com/artivis/manif/issues/181>`_
* fix missing return in const data()
* Merge pull request `#147 <https://github.com/artivis/manif/issues/147>`_ from artivis/fix/review_copy_constr_assign
  Review copy construstor/assignment
* fix some cppcheck
* fix use of MANIF_ASSERT
* add move semantic
* add macro MANIF_MOVE_NOEXCEPT
* review copy constr/assign se_2_3
* Merge branch 'devel' into fix/review_copy_constr_assign
* Add SE_2(3) Lie group (`#154 <https://github.com/artivis/manif/issues/154>`_)
* [SE(3)] Add setters for translation and rotation  (`#166 <https://github.com/artivis/manif/issues/166>`_)
* Merge pull request `#169 <https://github.com/artivis/manif/issues/169>`_ from GiulioRomualdi/patch-3
  Bugfix in SE3TangentBase::asSO3() function
* Fix undesired compile options when gcc is used (`#157 <https://github.com/artivis/manif/issues/157>`_)
  * Avoid to set compile options for gcc in the main CMakeLists.txt file
  * Set the required compile flags in the examples/CMakeLists.txt
* Fix Windows compilation (`#149 <https://github.com/artivis/manif/issues/149>`_)
  * Fix Windows compilation
* Add operator  scalar * tangent (`#153 <https://github.com/artivis/manif/issues/153>`_)
* cleanup eigen includes
* review copy constr/assign
* lt::optional explicit base constructor call
* Merge pull request `#138 <https://github.com/artivis/manif/issues/138>`_ from artivis/feature/manif_assert
  Add MANIF_ASSERT
* Merge pull request `#137 <https://github.com/artivis/manif/issues/137>`_ from artivis/feature/public_non_const_coeffs
  Add public non-const coeffs
* fix MANIF_ASSERT tests
* add MANIF_ASSERT
* add public non-const coeffs
* Merge pull request `#131 <https://github.com/artivis/manif/issues/131>`_ from artivis/feature/cheat-sheet
  Add theory cheat sheets
* add theory cheat sheets
* Merge pull request `#115 <https://github.com/artivis/manif/issues/115>`_ from artivis/fix/mem_alignment
  - Fix memory alignment issues
  - expand CI with cppcheck & valgrind
* Fix dependencies format. Fix typo in Lie name
* RandomEvaluatorImpl use proper return type
* add traits Base to all Map
* fix ceres-related tests memory alignment issues
* use memory alignment macro in all classes
* Merge pull request `#109 <https://github.com/artivis/manif/issues/109>`_ from artivis/feature/rn
  Add trivial groups Rn
* add MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND* macros
* MANIF\_*_TYPEDEF closer to abstract API
* GeneratorEvaluetor::run int -> unsigned int
* isApprox default eps
* do not install package manifest
* JOSS paper (`#30 <https://github.com/artivis/manif/issues/30>`_)
  Add JOSS paper.
  Co-authored-by: Joan Solà <jsola@iri.upc.edu>
* fix Vector typedef and add small test (`#118 <https://github.com/artivis/manif/issues/118>`_)
  fix `#117 <https://github.com/artivis/manif/issues/117>`_
* Contributors: Daniel S. Katz, Giulio Romualdi, Jeremie Deray, Joan Solà, Petter Nilsson, Prashanth Ramadoss, artivis
* JOSS paper (`#30 <https://github.com/artivis/manif/issues/30>`_)
  Add JOSS paper.
  Co-authored-by: Joan Solà <jsola@iri.upc.edu>
* fix Vector typedef and add small test (`#118 <https://github.com/artivis/manif/issues/118>`_)
  fix `#117 <https://github.com/artivis/manif/issues/117>`_
* Contributors: Jeremie Deray
