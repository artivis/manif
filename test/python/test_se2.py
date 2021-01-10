import numpy as np
import pytest

from manifpy import SE2, SE2Tangent


def test_constructor():
    state = SE2(4, 2, 1, 0)
    assert 4 == state.x()
    assert 2 == state.y()
    assert 1 == state.real()
    assert 0 == state.imag()
    assert 0 == state.angle()

    state = SE2(2, 4, 0.17)
    assert 2 == state.x()
    assert 4 == state.y()
    assert 0.17 == state.angle()

    state = SE2(4, 2, 1+0j)
    assert 4 == state.x()
    assert 2 == state.y()
    assert 0 == state.angle()

    state = SE2(np.array([2, 4]), 1+0j)
    assert 2 == state.x()
    assert 4 == state.y()
    assert 0 == state.angle()

    delta = SE2Tangent(4, 2, 0.17)
    assert 4 == delta.x()
    assert 2 == delta.y()
    assert 0.17 == delta.angle()

def test_accessors():
    state = SE2.Identity()

    assert 0 == state.x()
    assert 0 == state.y()
    assert 1 == state.real()
    assert 0 == state.imag()
    assert 0 == state.angle()

    delta = SE2Tangent.Zero()

    assert 0 == delta.x()
    assert 0 == delta.y()
    assert 0 == delta.angle()

def test_transform():
    state = SE2.Identity()
    transform = state.transform()

    assert (3, 3) == transform.shape
    assert (np.identity(3) == transform).all()

# def test_isometry():
#     state = SE2.Identity()
#     isometry = state.isometry()
#
#     assert (3, 3) == isometry.shape
#     assert (np.identity(3) == isometry).all()

def test_rotation():
    state = SE2.Identity()
    rotation = state.rotation()

    assert (2, 2) == rotation.shape
    assert (np.identity(2) == rotation).all()

def test_translation():
    state = SE2.Identity()
    translation = state.translation()

    assert (2,) == translation.shape
    assert (np.zeros(2,) == translation).all()
