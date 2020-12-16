# small helper function
function(manif_add_pytest target)
  add_test(
    NAME ${target}
    COMMAND ${PYTHON_EXECUTABLE} -m pytest ${target}.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  set_tests_properties(${target}
    PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}/python:$ENV{PYTHONPATH}"
  )
endfunction()

manif_add_pytest(pytest_manif)
manif_add_pytest(pytest_so2)
manif_add_pytest(pytest_se2)
manif_add_pytest(pytest_so3)
manif_add_pytest(pytest_se3)
manif_add_pytest(pytest_se_2_3)
