import numpy as np
import pytest

from manifpy import Quaternion


def test_constructor():
    q = Quaternion(1,2,3,4)
    assert 2 == q.x()
    assert 3 == q.y()
    assert 4 == q.z()
    assert 1 == q.w()

    q = Quaternion(np.array([1,2,3,4]))
    assert 1 == q.x()
    assert 2 == q.y()
    assert 3 == q.z()
    assert 4 == q.w()

    q = Quaternion(np.identity(3))
    assert 0 == q.x()
    assert 0 == q.y()
    assert 0 == q.z()
    assert 1 == q.w()

def test_vec():
    vec = Quaternion(1,2,3,4).vec()
    assert 2 == vec[0]
    assert 3 == vec[1]
    assert 4 == vec[2]

def test_identity():
    q = Quaternion.Identity()

    assert 0 == q.x()
    assert 0 == q.y()
    assert 0 == q.z()
    assert 1 == q.w()

    q_other = Quaternion()
    q_other.setIdentity()

    assert q.isApprox(q_other)
    assert q.isApprox(q_other * q_other)
    assert q.isApprox(q_other.normalized())

    q_inv = q.inverse()

    assert q_inv.isApprox(q_other)
    assert q_inv.isApprox(q.conjugate())

    assert (3,3) == q.toRotationMatrix().shape
    assert (np.identity(3) == q.toRotationMatrix()).all()

    assert (3,3) == q.matrix().shape
    assert (np.identity(3) == q.matrix()).all()

    assert 1 == q.norm()
    assert 1 == q.squaredNorm()
    assert 1 == q.dot(q) # cos(0)
    assert 0 == q.angularDistance(q)
