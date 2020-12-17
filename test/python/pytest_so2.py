import numpy as np
import pytest

from manifpy import SO2, SO2Tangent


def test_constructor():
    state = SO2(0.17)
    assert 0.17 == state.angle()

    state = SO2(1, 0)
    assert 0 == state.angle()

    delta = SO2Tangent(0.17)
    assert 0.17 == delta.angle()

def test_accessors():
    state = SO2.Identity()

    assert 1 == state.real()
    assert 0 == state.imag()
    assert 0 == state.angle()

    delta = SO2Tangent.Zero()

    assert 0 == delta.angle()

def test_transform():
    state = SO2.Identity()
    transform = state.transform()

    assert (3, 3) == transform.shape
    assert (np.identity(3) == transform).all()

def test_rotation():
    state = SO2.Identity()
    rotation = state.rotation()

    assert (2, 2) == rotation.shape
    assert (np.identity(2) == rotation).all()
