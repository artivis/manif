import numpy as np
import pytest

from manifpy import Quaternion, SO3, SO3Tangent


def test_constructor():
    state = SO3(0,0,0,1)
    assert 0 == state.x()
    assert 0 == state.y()
    assert 0 == state.z()
    assert 1 == state.w()

    state = SO3(0,0,0)
    assert 0 == state.x()
    assert 0 == state.y()
    assert 0 == state.z()
    assert 1 == state.w()

    state = SO3(Quaternion(1,0,0,0))
    assert 0 == state.x()
    assert 0 == state.y()
    assert 0 == state.z()
    assert 1 == state.w()

    # state = SO3(AngleAxis(0, UnitX()))
    # assert 0 == state.x()
    # assert 0 == state.y()
    # assert 0 == state.z()
    # assert 1 == state.w()

    # delta = SO3Tangent(1,2,3)
    # assert 1 == delta.x()
    # assert 2 == delta.y()
    # assert 3 == delta.z()

def test_accessors():
    state = SO3.Identity()

    assert 0 == state.x()
    assert 0 == state.y()
    assert 0 == state.z()
    assert 1 == state.w()

    delta = SO3Tangent.Zero()

    assert 0 == delta.x()
    assert 0 == delta.y()
    assert 0 == delta.z()

def test_transform():
    state = SO3.Identity()
    transform = state.transform()

    assert (4, 4) == transform.shape
    assert (np.identity(4) == transform).all()

def test_rotation():
    state = SO3.Identity()
    rotation = state.rotation()

    assert (3, 3) == rotation.shape
    assert (np.identity(3) == rotation).all()

def test_quaternion():
    state = SO3.Identity()
    quaternion = state.quat()

    assert Quaternion.Identity().isApprox(quaternion)
