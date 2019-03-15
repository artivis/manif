---
title: 'Manif: A micro Lie theory library for state-estimation in robotics applications'
tags:
  - Lie group
  - Lie algebra
  - State Estimation
  - Robotics
  - c++
authors:
 - name: Jérémie Deray
   orcid: 0000-0001-5279-8251
   affiliation: 1
 - name: Joan Solà
   orcid: 0000-0002-2933-3381
   affiliation: 1
affiliations:
 - name: Institut de Robòtica i Informàtica Industrial, CSIC-UPC, Llorens Artigas 4-6, 08028, Barcelona, Spain.
   index: 1
date: 2 December 2018
bibliography: paper.bib
---

# Summary

`manif` [@manif] is a micro Lie theory library targeted at
state estimation in robotics applications.
With a single dependency on `Eigen` [@eigenweb] and
a requirement on c++11 only, it is
developed as a header-only library making
it easy to integrate to existing projects.

There has been a remarkable effort in the last years in
the robotics community to formulate estimation problems
properly. This is motivated by an increasing demand for
precision, consistency and stability of the solutions.
Indeed, a proper modeling of the states and measurements,
the functions relating them, and their uncertainties,
is crucial to achieve these goals.
This has led to problem formulations involving what has been
known as ‘manifolds’, which in this context are no less
than the smooth topologic surfaces of the Lie groups where
the state representations evolve.

The `manif` library has been developed to make easily accessible
the most common operations on Lie groups in state estimation.
Its design is similar to that of `Eigen`, so that
all Lie group classes defined in `manif` have in common that
they inherit from a templated base class using static polymorphism.
This allows for the possibility the write generic code without
paying the price of pointer indirection.
Thanks to this polymorphism, the library is open to extensions to
Lie groups beyond the currently implemented SO(2), SE(2), SO(3) and SE(3).

The library is mathematically grounded in [@Sola18]
and often refers to it in the documentation,
especially for providing reference for the mathematical formulae.

# Related work

`Sophus` [@Sophus] C++ implementation of Lie Groups using `Eigen`.
Our work differs from `Sophus` in that all our classes inherit from
a common templated base class which enforces a common minimal API.
This allows for writing generic algorithms on Lie groups.
Moreover, the analytical Jacobian matrices are available to the user
for most of the operation on groups,
allowing complex chain of operations to be differentiated through the chain rule.
Jacobian matrices in `manif` are defined with respect to local
perturbations on the Lie group's tangent spaces,
whereas `Sophus` defines them with respect
to the representation vector that underlies the implementation.

`wave_geometry` [@wave_geometry] Manifold geometry with fast automatic derivatives
and coordinate frame semantics checking.
Our work differs from `wave_geometry` in that it relies on
c++11 which is still the standard in many laboratories and companies, while
`wave_geometry`, as of the time this paper is written,
requires a c++17-compatible compiler.
While both libraries rely on the external dependency `Eigen`,
`wave_geometry` also relies on Boost [@boostweb].
Finally, as of the time this paper is written, `wave_geometry` only implements
the groups SO(3) and SE(3) while `manif` also provides SO(2) and SE(2).

# References
