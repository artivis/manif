# small helper function
function(manif_add_pytest target)
  add_test(
    NAME ${target}
    COMMAND pytest-3 ${target}.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  set_tests_properties(${target}
    PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}:$ENV{PYTHONPATH}"
  )
endfunction()

manif_add_pytest(test_manif)
manif_add_pytest(test_so2)
manif_add_pytest(test_se2)
manif_add_pytest(test_so3)
manif_add_pytest(test_se3)
manif_add_pytest(test_se_2_3)
