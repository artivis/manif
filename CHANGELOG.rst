^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package manif
^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
