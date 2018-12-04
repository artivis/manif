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

Manif is a micro Lie algebra library targeted at 
state estimation in robotics application.
With a minimal dependency on [@eigenweb] and a requierement on cpp11 it is
developped as a header-only library making it easily integrable to existing project.

# Related work

[@Sophus] C++ implementation of Lie Groups using Eigen.

[@wave_geometry] Manifold geometry with fast automatic derivatives and coordinate frame semantics checking.

# References
