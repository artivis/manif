import numpy as np
import pytest

from PyManif import \
    R1, R1Tangent, \
    R2, R2Tangent, \
    R3, R3Tangent, \
    R4, R4Tangent, \
    R5, R5Tangent, \
    R6, R6Tangent, \
    R7, R7Tangent, \
    R8, R8Tangent, \
    R9, R9Tangent, \
    SO2, SO2Tangent, \
    SE2, SE2Tangent, \
    SO3, SO3Tangent, \
    SE3, SE3Tangent, \
    SE_2_3, SE_2_3Tangent


@pytest.mark.parametrize(
    "LieGroup, Tangent",
    [
     (R1, R1Tangent),
     (R2, R2Tangent),
     (R3, R3Tangent),
     (R4, R4Tangent),
     (R5, R5Tangent),
     (R6, R6Tangent),
     (R7, R7Tangent),
     (R8, R8Tangent),
     (R9, R9Tangent),
     (SO2, SO2Tangent),
     (SO3, SO3Tangent),
     (SE2, SE2Tangent),
     (SE3, SE3Tangent),
     (SE_2_3, SE_2_3Tangent),
    ]
)
class TestCommon:

    def test_Init(self, LieGroup, Tangent):
        state = LieGroup.Random()

        assert state != LieGroup.Identity()
        state = LieGroup.Identity()
        assert state == LieGroup.Identity()
        state.setRandom()
        assert state != LieGroup.Identity()
        state.setIdentity()
        assert state == LieGroup.Identity()

        delta = Tangent.Random()

        assert delta != Tangent.Zero()
        delta = Tangent.Zero()
        assert delta == Tangent.Zero()
        delta.setRandom()
        assert delta != Tangent.Zero()
        delta.setZero()
        assert delta == Tangent.Zero()

    def test_Assignment(self, LieGroup, Tangent):
        state = LieGroup.Random()
        state_other = LieGroup.Random()

        assert state != state_other

        state_other = state
        assert state == state_other

        delta = Tangent.Random()
        delta_other = Tangent.Random()

        assert delta != delta_other

        delta_other = delta
        assert delta == delta_other

        delta = Tangent(np.zeros(Tangent.DoF))
        assert (delta.coeffs() == np.zeros(Tangent.DoF)).all()
        data = np.random.rand(Tangent.DoF)
        delta = Tangent(data)
        assert (delta.coeffs() == data).all()
        assert not (delta.coeffs() == np.zeros(Tangent.DoF)).all()

    def test_Plus(self, LieGroup, Tangent):
        state = LieGroup.Random()
        delta = Tangent.Random()

        assert state.plus(delta) == (state + delta)
        assert state.plus(delta) == state.rplus(delta)
        assert state == state + Tangent.Zero()
        assert state == Tangent.Zero() + state
        assert LieGroup.Identity() == LieGroup.Identity() + Tangent.Zero()

    def test_Minus(self, LieGroup, Tangent):
        state = LieGroup.Random()
        other = LieGroup.Random()
        delta = Tangent.Random()

        assert state.minus(other) == state - other
        assert state.minus(other) == state.rminus(other)
        assert Tangent.Zero() == state - state
        assert Tangent.Zero() == delta - delta

    # def test_plusEq(self, LieGroup, Tangent):
    #     state = LieGroup.Random()
    #     copy = state
    #     delta = Tangent.Random()
    #
    #     state += delta
    #
    #     assert copy + delta == state

    def test_Compose(self, LieGroup, Tangent):
        state = LieGroup.Random()
        other = LieGroup.Random()

        assert state.compose(other) == state * other
        assert state.compose(LieGroup.Identity()) == state
        assert LieGroup.Identity().compose(state) == state
        assert LieGroup.Identity() == state.compose(state.inverse())
        assert LieGroup.Identity() == state.inverse().compose(state)

    def test_Exp(self, LieGroup, Tangent):
        assert LieGroup.Identity() == Tangent.Zero().exp()

    def test_Log(self, LieGroup, Tangent):
        assert Tangent.Zero() == LieGroup.Identity().log()

    def test_LogExp(self, LieGroup, Tangent):
        state = LieGroup.Random()

        assert state == state.log().exp()

    def test_ExpLog(self, LieGroup, Tangent):
        delta = Tangent.Random()

        state = delta.exp()
        delta_other = state.log()
        print(delta)
        print(delta.exp().log())
        print(delta_other)

        assert delta == delta_other

    def test_Between(self, LieGroup, Tangent):
        state = LieGroup.Random()

        assert LieGroup.Identity() == state.between(state)

    def test_Random(self, LieGroup, Tangent):
        assert LieGroup.Random() != LieGroup.Random()
        assert Tangent.Random() != Tangent.Random()

    def test_isApprox(self, LieGroup, Tangent):
        state = LieGroup.Random()
        other = LieGroup.Random()

        assert state.isApprox(state)
        assert not state.isApprox(other)

        state = Tangent.Random()
        other = Tangent.Random()

        assert state.isApprox(state)
        assert not state.isApprox(other)

        assert LieGroup.Identity().isApprox(LieGroup.Identity())
        assert Tangent.Zero().isApprox(Tangent.Zero())
        assert Tangent.Zero().isApprox(np.zeros(Tangent.DoF))

    def test_Inner(self, LieGroup, Tangent):
        assert 0 == Tangent.Zero().weightedNorm()
        assert 0 == Tangent.Zero().squaredWeightedNorm()
        assert 0 == Tangent.Zero().inner(Tangent.Zero())

        delta = Tangent.Random()
        delta_other = Tangent.Random()
        assert delta.squaredWeightedNorm() == delta.inner(delta)
        assert delta.inner(delta_other) == delta_other.inner(delta)

    def test_Act(self, LieGroup, Tangent):
        state = LieGroup.Identity()
        point = np.random.rand(Tangent.Dim)

        pout = state.act(point)

        assert (point == pout).all()

        state = LieGroup.Random()

        pout = state.act(point)

        assert not (point == pout).all()

        pout = state.inverse().act(pout)

        # allclose: absolute(a - b) <= (1e-08 + 1e-05 * absolute(b))
        assert np.allclose(point, pout)

    # def test_smallAdj(self, LieGroup, Tangent):
    #     delta = Tangent.Random()
    #     delta_other = Tangent.Random()
    #
    #     assert (delta.smallAdj() * delta_other).hat() == delta.hat() * delta_other.hat() - delta_other.hat() * delta.hat()

    def test_InverseJac(self, LieGroup, Tangent):

        state = LieGroup.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)

        # https://pybind11.readthedocs.io/en/stable/advanced/cast/eigen.html#storage-orders
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF), order='F')

        state_out = state.inverse(J_sout_s)

        print(type(J_sout_s))
        print(J_sout_s)
        print(type(w))
        print(w)
        # print(type(J_sout_s))
        # print(J_sout_s*w)

        state_pert = (state+w).inverse()
        # state_lin  = state_out.rplus(J_sout_s*w)

        # assert state_pert == state_lin
