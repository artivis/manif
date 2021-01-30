# Building the documentation

We detail here how to build the documentation locally.
The overall documentation relies on two tools,
namely [Doxygen][doxygen] and [m.css][mcss].
The former is mainly used to build the C++ API documentation
while the former is used to generate the Python API documentation
and the overall documentation website.

See how to build each below.

## mcss

```terminal
python3 -m pip install jinja2 Pygments
sudo apt install -y doxygen graphicsmagick-imagemagick-compat
git clone git://github.com/mosra/m.css
```

## C++ API

The C++ API documentation relies only on Doxygen.
If you are fine with the antiquated look of a Doxygen-based
website, here is how to build it.

First let's make sure doxygen is installed,

```terminal
apt install -y doxygen
```

Let us now build to doc,

```terminal
cd manif/docs
doxygen Doxyfile
```

You can now explore the website by opening the file
`manif/docs/html/index.html` in your web browser.

## Python API

```terminal
cd manif/docs
python3 ~/path/to/m.css/documentation/doxygen.py conf_cpp.py
```

## Building the website

```terminal
cd manif/docs
mkdir -p site/cpp
```

```terminal
python3 ~/path/to/m.css/documentation/python.py conf_python.py
python3 ~/path/to/m.css/documentation/doxygen.py conf_cpp.py
python3 ~/path/to/m.css/documentation/doxygen.py conf.py
```

[doxygen]: https://www.doxygen.nl/index.html
[mcss]: https://mcss.mosra.cz/