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

A Lie group is an old mathematical abstract object
dating back to the XIX century, when mathematician Sophus Lie
laid the foundations of the theory of continuous transformation
groups.
There has been a remarkable effort in the last years in
the robotics community to formulate estimation problems
properly. This is motivated by an increasing demand for
precision, consistency and stability of the solutions. Indeed, a
proper modeling of the states and measurements, the functions
relating them, and their uncertainties, is crucial to achieve
these goals. This has led to designs involving what has been
known as ‘manifolds’, which in this context are no less
than the smooth topologic surfaces of the Lie groups where
the state representations evolve. Relying on the Lie theory
we are able to construct a rigorous calculus corpus to
handle uncertainties, derivatives and integrals with precision
and ease. Typically, these works have focused on the well
known manifolds of rotation SO(3) and rigid motion SE(3).

`manif` is a micro Lie theory library targeted at 
state estimation in robotics application.
With a minimal dependency on [@eigenweb] and a requierement on c++11 only it is
developped as a header-only library making it easily integrable to existing project.

# Manif

The `manif` library design is similar to that of [@eigenweb]. 
All Lie group classes defined in `manif` have in common that they inherit from a templated base class.
The `manif` library is mathematically grounded in [@Sola18] and often refers to it in it documentation
especially for mathematical developments.

# Related work

[@Sophus] C++ implementation of Lie Groups using Eigen.
Our work differs from [@Sophus] in that all our classes inherit from a common templated base class
which ensure a common minimal API. This allows for writing generic algorithms on Lie groups.
Moreover, the Jacobian matrix are available to the user for most of the operation on groups
allowing complex chain of operations to be differentiate through the chain rule.

[@wave_geometry] Manifold geometry with fast automatic derivatives and coordinate frame semantics checking.
Our work differs from [@wave_geometry] in that it offers analitically computed Jacobian matrix whereas
[@wave_geometry] relies on automatic-differentiation.

Finally, Jacobian matrix in `manif` are defined with respect to a local perturbation on the Lie groups
unlink both [@Sophus] and [@wave_geometry] which defines them with respect 
to the representation vector that underlie the implemtation.

# References
