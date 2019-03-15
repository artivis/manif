---
title: 'Manif: A micro Lie theory library for state-estimation in robotics applications'
tags:
  - Lie group
  - Lie algebra
  - State Estimation
  - Robotics
  - c++
authors:
 - name: Jeremie Deray
   orcid: 0000-0001-5279-8251
   affiliation: 1
 - name: Joan Sola
   orcid: 0000-0002-2933-3381
   affiliation: 1
affiliations:
 - name: Institut de Rob\`otica i Inform\`atica Industrial, CSIC-UPC, Llorens Artigas 4-6, 08028, Barcelona, Spain.
   index: 1
date: 2 December 2018
bibliography: paper.bib
---

# Summary

`manif` [@manif] is a micro Lie theory library targeted at
state estimation in robotics applications.
With a single dependency on `Eigen` [@eigenweb] and
a requirement on c++11 only it is
developed as a header-only library making
it easily to integrate to existing projects.

There has been a remarkable effort in the last years in
the robotics community to formulate estimation problems
properly. This is motivated by an increasing demand for
precision, consistency and stability of the solutions.
Indeed, a proper modeling of the states and measurements,
the functions relating them, and their uncertainties,
is crucial to achieve these goals.
It has led to problems formulation involving what has been
known as ‘manifolds’, which in this context are no less
than the smooth topologic surfaces of the Lie groups where
the state representations evolve.

The `manif` library has been developed to make easily accessible
the most common operations on Lie groups in state estimation.
Its design is similar to that of `Eigen` so that
all Lie group classes defined in `manif` have in common that
they inherit from a templated base class using static polymorphism.
This allows for the possibility the write generic code without
paying the price of pointer indirection.

The library is mathematically grounded in [@Sola18]
and often refers to it in it documentation
especially for mathematical developments.

# Related work

`Sophus` [@Sophus] C++ implementation of Lie Groups using `Eigen`.
Our work differs from `Sophus` in that all our classes inherit from
a common templated base class which enforces a common minimal API.
This allows for writing generic algorithms on Lie groups.
Moreover, the analytical Jacobian matrices are available to the user
for most of the operation on groups,
allowing complex chain of operations to be differentiated through the chain rule.
Jacobian matrices in `manif` are defined with respect to a local
perturbation on the Lie group's tangent space,
whereas `Sophus` defines them with respect
to the representation vector that underlie the implementation.

`wave_geometry` [@wave_geometry] Manifold geometry with fast automatic derivatives
and coordinate frame semantics checking.
Our work differs from `wave_geometry` in that it relies on
c++11 which is still the standard in many laboratories and companies while
`wave_geometry`, as of the time this paper is written,
requires a c++17-compatible compiler.
While both library rely on the external dependency is `Eigen`,
`wave_geometry` also relies on Boost [@boostweb].
Finally, as of the time this paper is written, `wave_geometry` only implements
the groups $SO3$ and $SE3$ while `manif` also provides $SO2$ and $SE2$

# References
