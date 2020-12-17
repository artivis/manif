import numpy as np
import pytest

from manifpy import SE_2_3, SE_2_3Tangent


def test_constructor():
    state = SE_2_3(0,0,0, 0,0,0, 0,0,0)
    assert 0 == state.x()
    assert 0 == state.y()
    assert 0 == state.z()
    # assert 0 == state.quat()
    assert 0 == state.vx()
    assert 0 == state.vy()
    assert 0 == state.vz()

    # state = SE_2_3(np.array([1,2,3]), Quaternion(1,0,0,0), np.array([4,5,6]))
    # assert 1 == state.x()
    # assert 2 == state.y()
    # assert 3 == state.z()
    # assert 1 == state.quat()
    # assert 4 == state.vx()
    # assert 5 == state.vy()
    # assert 6 == state.vz()

    # state = SE_2_3(np.array([1,2,3]), AngleAxis(0, UnitX()), np.array([4,5,6]))
    # assert 1 == state.x()
    # assert 2 == state.y()
    # assert 3 == state.z()
    # assert 1 == state.w()
    # assert 4 == state.vx()
    # assert 5 == state.vy()
    # assert 6 == state.vz()

    # delta = SE_2_3Tangent(1,2,3)
    # assert 1 == delta.x()
    # assert 2 == delta.y()
    # assert 3 == delta.z()

def test_accessors():
    state = SE_2_3.Identity()
    assert 0 == state.x()
    assert 0 == state.y()
    assert 0 == state.z()
    # assert 0 == state.quat()
    assert 0 == state.vx()
    assert 0 == state.vy()
    assert 0 == state.vz()

    delta = SE_2_3Tangent.Zero()

    # assert 0 == delta.x()
    # assert 0 == delta.y()
    # assert 0 == delta.z()

# def test_isometry():
#     state = SE_2_3.Identity()
#     isometry = state.isometry()
#
#     assert (4, 4) == isometry.shape
#     assert (np.identity(4) == isometry).all()

def test_rotation():
    state = SE_2_3.Identity()
    rotation = state.rotation()

    assert (3, 3) == rotation.shape
    assert (np.identity(3) == rotation).all()

# def test_quaternion():
#     state = SO3.Identity()
#     quaternion = state.quaternion()
#
#     assert (4,) == quaternion.shape
#     assert (np.zeros(4,) == quaternion).all()

def test_translation():
    state = SE_2_3.Identity()
    translation = state.translation()

    assert (3,) == translation.shape
    assert (np.zeros(3,) == translation).all()
