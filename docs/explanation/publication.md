# Papers & courses

The **manif** project stems out of a larger body of work on the topic of state parameterization and Lie groups.

Most notably, it accompanies the paper `"A micro Lie theory for state estimation in robotics", J. Sola, J. Deray, D. Atchuthan`, presented [below](#micro-lie-theory).

## Lie group cheat sheets

In a rush? Check out our Lie group theory take away,
the [Lie group cheat sheet][cheat-sheet] (or hit [dowload][cheat-sheet-download]).

## Micro Lie theory

Find the accompanying paper readily available on Arxiv:

[`"A micro Lie theory for state estimation in robotics", J. Sola, J. Deray, D. Atchuthan`](https://arxiv.org/abs/1812.01537).

### Abstract

A Lie group is an old mathematical abstract object dating back to the XIX century, when mathematician Sophus Lie laid the foundations of the theory of continuous transformation groups. As it often happens, its usage has spread over diverse areas of science and technology many years later. In robotics, we are recently experiencing an important trend in its usage, at least in the fields of estimation, and particularly in motion estimation for navigation. Yet for a vast majority of roboticians, Lie groups are highly abstract constructions and therefore difficult to understand and to use. This may be due to the fact that most of the literature on Lie theory is written by and for mathematicians and physicists, who might be more used than us to the deep abstractions this theory deals with.
In estimation for robotics it is often not necessary to exploit the full capacity of the theory, and therefore an effort of selection of materials is required. In this paper, we will walk through the most basic principles of the Lie theory, with the aim of conveying clear and useful ideas, and leave a significant corpus of the Lie theory behind. Even with this mutilation, the material included here has proven to be extremely useful in modern estimation algorithms for robotics, especially in the fields of SLAM, visual odometry, and the like.
Alongside this micro Lie theory, we provide a chapter with a few application examples, and a vast reference of formulas for the major Lie groups used in robotics, including most jacobian matrices and the way to easily manipulate them. We also present a new C++ template-only library implementing all the functionality described here.

### Bibetex

```latex
@techreport{SOLA-18-Lie,
    Address = {Barcelona},
    Author = {Joan Sol\`a and Jeremie Deray and Dinesh Atchuthan},
    Institution = {{Institut de Rob\`otica i Inform\`atica Industrial}},
    Number = {IRI-TR-18-01},
    Title = {A micro {L}ie theory for state estimation in robotics},
    Howpublished="\url{http://arxiv.org/abs/1812.01537}",
    Year = {2018}
}
```

## manif

The **manif** project is also subject of a publication at the Journal of Open Source Software (JOSS):

[`"Manif: A micro Lie theory library for state estimation in robotics applications" J. Deray, J. Sola`][deray20].

If you use this software,
please consider citing it!

### Abstract

There has been a remarkable effort in the last years in the robotics community to formulate
estimation problems properly (Eade, 2013)(Barfoot, 2017). This is motivated by an increasing
demand for precision, consistency, and stability of the solutions. Indeed, proper modeling of
the states and measurements, the functions relating them, and their uncertainties, is crucial
to achieve these goals. This has led to problem formulations involving what has been known
as ‘manifolds’, which in this context are no less than the smooth topologic surfaces of the Lie
groups where the state representations evolve (Chirikjian, 2011).
manif (Deray & Solà, 2019) is a micro Lie theory library targeted at state estimation in
robotics applications. With a single dependency on Eigen (Guennebaud, Jacob, & others,
2010) and a requirement on C++11 only, it is developed as a header-only library, making it
easy to integrate to existing projects.
The manif library provides simple interfaces to the most common operations on Lie groups
in state estimation. Its design is similar to Eigen, in that all Lie group classes inherit from a
templated base class using static polymorphism. This allows for writing generic code without
paying the price of pointer arithmetic. Thanks to this polymorphism, the library is open to
extensions to Lie groups beyond the currently implemented: the Special Orthogonal groups
SO(2) and SO(3) and the Special Euclidean groups SE(2) and SE(3).
The mathematical foundations of the library are given in (Solà, Deray, & Atchuthan, 2018),
which is often referred to in the documentation, especially for providing references for the
mathematical formulae.

### Bibetex

```latex
@article{Deray-20-JOSS,
  doi = {10.21105/joss.01371},
  url = {https://doi.org/10.21105/joss.01371},
  year = {2020},
  publisher = {The Open Journal},
  volume = {5},
  number = {46},
  pages = {1371},
  author = {Jérémie Deray and Joan Solà},
  title = {Manif: A micro {L}ie theory library for state estimation in robotics applications},
  journal = {Journal of Open Source Software}
}
```

## Videos

Several courses and workshops were also delivered along the years.
A selection is proposed here.

### Robotics & AI Summer School 2022

```{youtube} csolG83gCV8
```

### IROS'20 Workshop on Bringing Geometric Methods to Robot Learning, Optimization and Control

```{youtube} QR1p0Rabuww
```

### Robotics & AI Summer School 2020

```{youtube} nHOcoIyJj2o
```

[cheat-sheet]: https://github.com/artivis/manif/blob/devel/paper/Lie_theory_cheat_sheet.pdf
[cheat-sheet-download]: https://github.com/artivis/manif/raw/devel/paper/Lie_theory_cheat_sheet.pdf
[deray20]: https://joss.theoj.org/papers/10.21105/joss.01371
