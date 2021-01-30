# Quick start

- [Quick start](#quick-start)
  - [Getting Pybind11](#getting-pybind11)
  - [Installation](#installation)
    - [Dependencies](#dependencies)
    - [From source](#from-source)
  - [Use manifpy in your project](#use-manifpy-in-your-project)
  - [Tutorials and application demos](#tutorials-and-application-demos)

## Getting Pybind11

The Python wrappers are generated using [pybind11][pybind11-rtd]. So first we need to install it,
but we want it available directly in our environment root so that `CMake` can find it.
To do so we can use,

```bash
python3 -m pip install "pybind11[global]"
```

Note that this is not recommended when using one's system Python,
as it will add files to `/usr/local/include/pybind11` and `/usr/local/share/cmake/pybind11`.

Another way is to use `CMake` to install it,

```bash
git clone https://github.com/pybind/pybind11.git
cd pybind11 && mkdir build && cd build
cmake ..
make install
```

## Installation

### Dependencies

- Eigen 3 :
  - Linux ( Ubuntu and similar )

      ```bash
      apt-get install libeigen3-dev
      ```

  - OS X

      ```bash
      brew install eigen
      ```

- [lt::optional][optional-repo] : included in the `external` folder

Python bindings also depends on `numpy`.

```bash
python3 -m pip install -r requirements
```

### From source

To generate `manif` Python bindings run,

```bash
git clone https://github.com/artivis/manif.git
cd manif
python3 -m pip install .
```

## Use manifpy in your project

```python
from manifpy import SE3

...

state = SE3.Identity()

...
```

## Tutorials and application demos

We provide some self-contained and self-explained executables implementing some real problems.
Their source code is located in `manif/examples/`.
These demos are:

- [`se2_localization.py`](examples/se2_localization.py): 2D robot localization based on fixed landmarks using SE2 as robot poses. This implements the example V.A in the paper.
- [`se3_localization.py`](examples/se3_localization.py): 3D robot localization based on fixed landmarks using SE3 as robot poses. This re-implements the example above but in 3D.
- [`se2_sam.py`](examples/se2_sam.py): 2D smoothing and mapping (SAM) with simultaneous estimation of robot poses and landmark locations, based on SE2 robot poses. This implements a the example V.B in the paper.
- [`se3_sam.py`](examples/se3_sam.py): 3D smoothing and mapping (SAM) with simultaneous estimation of robot poses and landmark locations, based on SE3 robot poses. This implements a 3D version of the example V.B in the paper.
- [`se3_sam_selfcalib.py`](examples/se3_sam_selfcalib.py): 3D smoothing and mapping (SAM) with self-calibration, with simultaneous estimation of robot poses, landmark locations and sensor parameters, based on SE3 robot poses. This implements a 3D version of the example V.C in the paper.
- [`se_2_3_localization.py`](examples/se_2_3_localization.py): A strap down IMU model based 3D robot localization, with measurements of fixed landmarks, using SE_2_3 as extended robot poses (translation, rotation and linear velocity).

To run a demo, simply go to the `manif/examples/` folder and run,

```bash
cd manif/examples
python3 se2_localization.py
```

[//]: # (URLs)

[pybind11-rtd]: https://pybind11.readthedocs.io/en/stable/index.html
[optional-repo]: https://github.com/TartanLlama/optional
