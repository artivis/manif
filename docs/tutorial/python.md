# Getting started in Python3

**manif** provides Python3 bindings,
called **manifpy**,
allowing to prototype and develop directly in Python3.

## Installation

### From conda

**manifpy** can be installed directly from [conda-forge][conda-manifpy]:

```bash
conda install -c conda-forge manifpy
```

### From source

The Python3 wrappers are generated using [pybind11][pybind11-rtd].
So first, we need to install it,
however it must be available directly in the environment root so that `CMake` can find it.

To do so use:

```bash
python3 -m pip install "pybind11[global]"
```

Prior to installing **manifpy**,
we have to install its main dependency `Eigen3`:

`````{tab-set}
````{tab-item} Ubuntu
:sync: key1
```bash
apt-get install libeigen3-dev
```
````
````{tab-item} OS X
:sync: key2
```bash
brew install eigen
```
````
`````

Next, clone the repository locally if you haven't done so already:

```bash
git clone https://github.com/artivis/manif.git
cd manif
```

To buid and install **manif**,
use the following commands:

```bash
python3 -m pip install .
```

## Use manif in a project

```python
from manifpy import SE3

...

state = SE3.Identity()

...
```

From there, have a look at the howtos for further informations
as well as the [examples](../reference/examples-python.md) to find several self-contained and self-explained examples implementing some real problems.

[//]: # (URLs)

[pybind11-rtd]: https://pybind11.readthedocs.io/en/stable/index.html
[conda-manifpy]: https://anaconda.org/conda-forge/manifpy
