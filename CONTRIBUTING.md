# Contributing

## General guidelines

**manif** is developed according to Vincent Driessen's [Gitflow Workflow][git-workflow].
This means,

- the `master` branch is for releases only.
- development is done on feature branches.
- finished features are integrated via PullRequests into the branch `devel`.

For a PullRequest to get merged into `devel`, it must pass

- Review by one of the maintainers.
  - Are the changes introduced in scope of **manif**?
  - Is the documentation updated?
  - Are enough reasonable tests added?
  - Will these changes break the API?
  - Do the new changes follow the current style of naming?
- Compile / Test / Run on all target environments.

Note: The test suite detailed below is run in CI for many targets environments including,

- Ubuntu 16.04/18.04/20.04
- MacOS 10.15
- Visual Studio 15

## Development environment

We will detail here how to set up a development environment.
It is recommended to use containers if you do not want to install the dependencies on your host.
You may refer to [this blog post](lxd-post) to set up a LXD container.

First let us clone the **manif** repo,

```terminal
git clone https://github.com/artivis/manif.git
cd manif
```

Let's install all dependencies for development and testing,

```terminal
apt install libeigen3-dev
python3 -m pip install "pybind11[global]" pytest
python3 -m pip install -r requirements
```

We can now build **manif**, its Python wrappers and all tests,

```terminal
mkdir build && cd build
cmake -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON -DBUILD_PYTHON_BINDINGS=ON -DBUILD_EXAMPLES=ON ..
make
```

To run the C++ tests execute the following,

```terminal
ctest --output-on-failure
```

To run the Python tests,

```terminal
cd manif
pytest
```

[//]: # (URLs)

[git-workflow]: http://nvie.com/posts/a-successful-git-branching-model
[lxd-post]: https://artivis.github.io/post/2020/lxc