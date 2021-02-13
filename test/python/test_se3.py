import numpy as np
import pytest

from manifpy import SE3, SE3Tangent


def test_constructor():
    state = SE3(0,0,0,0,0,0)
    assert 0 == state.x()
    assert 0 == state.y()
    assert 0 == state.z()
    # assert 1 == state.quat()

    # state = SE3(np.array([1,2,3]), Quaternion(1,0,0,0))
    # assert 0 == state.x()
    # assert 0 == state.y()
    # assert 0 == state.z()

    # state = SE3(np.array([1,2,3]), AngleAxis(0, UnitX()))
    # assert 0 == state.x()
    # assert 0 == state.y()
    # assert 0 == state.z()

    # delta = SE3Tangent(1,2,3,4,5,6)
    # assert 1 == delta.x()
    # assert 2 == delta.y()
    # assert 3 == delta.z()

def test_accessors():
    state = SE3.Identity()

    assert 0 == state.x()
    assert 0 == state.y()
    assert 0 == state.z()

    delta = SE3Tangent.Zero()

    # assert 0 == delta.x()
    # assert 0 == delta.y()
    # assert 0 == delta.z()

def test_transform():
    state = SE3.Identity()
    transform = state.transform()

    assert (4, 4) == transform.shape
    assert (np.identity(4) == transform).all()

def test_rotation():
    state = SE3.Identity()
    rotation = state.rotation()

    assert (3, 3) == rotation.shape
    assert (np.identity(3) == rotation).all()

# def test_quaternion():
#     state = SO3.Identity()
#     quaternion = state.quaternion()
#
#     assert (4,) == quaternion.shape
#     assert (np.zeros(4,) == quaternion).all()

# def test_translation():
#     state = SE3.Identity()
#     translation = state.translation()
#
#     assert (3,) == translation.shape
#     assert (np.zeros(3,) == translation).all()
