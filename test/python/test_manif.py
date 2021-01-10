import numpy as np
import pytest

from manifpy import \
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

    # def test_ExpLog(self, LieGroup, Tangent):
    #     delta = Tangent.Random()

    #     state = delta.exp()
    #     delta_other = state.log()
    #     print(delta)
    #     print(delta.exp().log())
    #     print(delta_other)

    #     assert delta == delta_other

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

    def test_smallAdj(self, LieGroup, Tangent):

        if LieGroup in (SO2, R1):
            pytest.skip("hat is a scalar (Dim 1), numpy doesn't support matmul")

        delta = Tangent.Random()
        delta_other = Tangent.Random()

        assert np.allclose(
            (delta.smallAdj() * delta_other).hat(),
            delta.hat() @ delta_other.hat() - delta_other.hat() @ delta.hat()
        )

    def test_InverseJac(self, LieGroup, Tangent):
        state = LieGroup.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF))

        state_out = state.inverse(J_sout_s)

        state_pert = (state+w).inverse()
        state_lin  = state_out.rplus(J_sout_s * w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

    def test_LogJac(self, LieGroup, Tangent):
        state = LieGroup.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF))

        state_out = state.log(J_sout_s)

        state_pert = (state+w).log()
        state_lin  = state_out + (J_sout_s*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

    def test_ExpJac(self, LieGroup, Tangent):
        delta = Tangent.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF))

        state_out = delta.exp(J_sout_s)

        state_pert = (delta+w).exp()
        state_lin  = state_out + (J_sout_s*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

        delta.setZero()
        state_out = delta.exp(J_sout_s)

        state_pert = (delta+w).exp()
        state_lin  = state_out + (J_sout_s*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

    def test_ComposeJac(self, LieGroup, Tangent):
        state = LieGroup.Random()
        state_other = LieGroup.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF))
        J_sout_so = np.zeros((LieGroup.DoF, LieGroup.DoF))

        state_out = state.compose(state_other, J_sout_s, J_sout_so)

        state_pert = (state+w).compose(state_other)
        state_lin  = state_out + J_sout_s*w

        assert state_pert.isApprox(state_lin, eps=1e-7)

        state_pert = state.compose(state_other+w)
        state_lin  = state_out + J_sout_so*w

        assert state_pert.isApprox(state_lin, eps=1e-7)

        #

        state_out = state.compose(state_other, J_out_self = J_sout_s, J_out_other = J_sout_so)

        state_pert = (state+w).compose(state_other)
        state_lin  = state_out + J_sout_s*w

        assert state_pert.isApprox(state_lin, eps=1e-7)

        state_pert = state.compose(state_other+w)
        state_lin  = state_out + J_sout_so*w

        assert state_pert.isApprox(state_lin, eps=1e-7)

        #

        state_out = state.compose(state_other, J_out_other = J_sout_so, J_out_self = J_sout_s)

        state_pert = (state+w).compose(state_other)
        state_lin  = state_out + J_sout_s*w

        assert state_pert.isApprox(state_lin, eps=1e-7)

        state_pert = state.compose(state_other+w)
        state_lin  = state_out + J_sout_so*w

        assert state_pert.isApprox(state_lin, eps=1e-7)

        #

        state_out = state.compose(state_other, J_out_self = J_sout_s)

        state_pert = (state+w).compose(state_other)
        state_lin  = state_out + J_sout_s*w

        assert state_pert.isApprox(state_lin, eps=1e-7)

        #

        state_out = state.compose(state_other, J_out_other = J_sout_so)

        state_pert = state.compose(state_other+w)
        state_lin  = state_out + J_sout_so*w

        assert state_pert.isApprox(state_lin, eps=1e-7)

    def test_BetweenJac(self, LieGroup, Tangent):
        state = LieGroup.Random()
        state_other = LieGroup.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF))
        J_sout_so = np.zeros((LieGroup.DoF, LieGroup.DoF))

        state_out = state.between(state_other, J_sout_s, J_sout_so)

        state_pert = (state + w).between(state_other)
        state_lin  = state_out + (J_sout_s * w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

        state_pert = state.between(state_other + w)
        state_lin  = state_out + (J_sout_so * w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

    def test_RplusJac(self, LieGroup, Tangent):
        state = LieGroup.Random()
        delta = Tangent.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF))
        J_sout_t = np.zeros((LieGroup.DoF, LieGroup.DoF))

        state_out = state.rplus(delta, J_sout_s, J_sout_t)

        state_pert = (state+w).rplus(delta)
        state_lin  = state_out.rplus(J_sout_s*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

        state_pert = state.rplus(delta+w)
        state_lin  = state_out.rplus(J_sout_t*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

    def test_LplusJac(self, LieGroup, Tangent):
        state = LieGroup.Random()
        delta = Tangent.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF))
        J_sout_t = np.zeros((LieGroup.DoF, LieGroup.DoF))

        state_out = state.lplus(delta, J_sout_s, J_sout_t)

        state_pert = (state+w).lplus(delta)
        state_lin  = state_out.rplus(J_sout_s*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

        state_pert = state.lplus(delta+w)
        state_lin  = state_out.rplus(J_sout_t*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

    def test_PlusJac(self, LieGroup, Tangent):
        state = LieGroup.Random()
        delta = Tangent.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF))
        J_sout_t = np.zeros((LieGroup.DoF, LieGroup.DoF))

        state_out = state.plus(delta, J_sout_s, J_sout_t)

        state_pert = (state+w).plus(delta)
        state_lin  = state_out.plus(J_sout_s*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

        state_pert = state.plus(delta+w)
        state_lin  = state_out.plus(J_sout_t*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

    def test_RminusJac(self, LieGroup, Tangent):
        state = LieGroup.Random()
        state_other = LieGroup.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF))
        J_sout_so = np.zeros((LieGroup.DoF, LieGroup.DoF))

        state_out = state.rminus(state_other, J_sout_s, J_sout_so)

        state_pert = (state+w).rminus(state_other)
        state_lin  = state_out.plus(J_sout_s*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

        state_pert = state.rminus(state_other+w)
        state_lin  = state_out.plus(J_sout_so*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

    def test_LminusJac(self, LieGroup, Tangent):
        state = LieGroup.Random()
        state_other = LieGroup.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF))
        J_sout_so = np.zeros((LieGroup.DoF, LieGroup.DoF))

        state_out = state.lminus(state_other, J_sout_s, J_sout_so)

        state_pert = (state+w).lminus(state_other)
        state_lin  = state_out.plus(J_sout_s*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

        state_pert = state.lminus(state_other+w)
        state_lin  = state_out.plus(J_sout_so*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

    def test_MinusJac(self, LieGroup, Tangent):
        state = LieGroup.Random()
        state_other = LieGroup.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        J_sout_s = np.zeros((LieGroup.DoF, LieGroup.DoF))
        J_sout_so = np.zeros((LieGroup.DoF, LieGroup.DoF))

        state_out = state.minus(state_other, J_sout_s, J_sout_so)

        state_pert = (state+w).minus(state_other)
        state_lin  = state_out.plus(J_sout_s*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

        state_pert = state.minus(state_other+w)
        state_lin  = state_out.plus(J_sout_so*w)

        assert state_pert.isApprox(state_lin, eps=1e-7)

    def test_Adj(self, LieGroup, Tangent):
        state = LieGroup.Random()
        state_other = LieGroup.Random()
        delta = Tangent.Random()

        Adja = state.adj()
        Adjb = state_other.adj()
        Adjc = state.compose(state_other).adj()

        # assert ((Adja @ Adjb) == Adjc).all()

        assert np.allclose(Adja @ Adjb, Adjc)

        assert state + delta == state.adj() * delta + state

        if LieGroup in (SO2, R1):
            pytest.skip("Adj is a scalar (Dim 1), numpy doesn't support inversion")

        assert np.allclose(np.linalg.inv(state.adj()), state.inverse().adj())

    def test_Adj2(self, LieGroup, Tangent):

        if LieGroup in (SO2, R1):
            pytest.skip("Jr/Jl/Adj are scalar (Dim 1), numpy doesn't support matmul")

        state = LieGroup.Random()

        Adj = state.adj()
        tan = state.log()

        Jr = tan.rjac()
        Jl = tan.ljac()

        assert np.allclose(Jl, Adj @ Jr)
        assert np.allclose(Adj, Jl @ np.linalg.inv(Jr))
        assert np.allclose(Jl, (-tan).rjac())

        state.setIdentity()

        Adj = state.adj()
        tan = state.log()

        Jr = tan.rjac()
        Jl = tan.ljac()

        assert np.allclose(Jl, Adj @ Jr)
        assert np.allclose(Adj, Jl @ np.linalg.inv(Jr))
        assert np.allclose(Jl, (-tan).rjac())

    @pytest.mark.skip(reason="invrjac/invljac not implemented yet")
    def test_JrJrinvJlJlinv(self, LieGroup, Tangent):
        state = LieGroup.Random()

        tan = state.log()
        Jr = tan.rjac()
        Jl = tan.ljac()

        Jrinv = tan.rjacinv()
        Jlinv = tan.ljacinv()

        I = np.identity(LieGroup.DoF)

        assert I == Jr @ Jrinv
        assert I == Jl @ Jlinv

    def test_ActJac(self, LieGroup, Tangent):
        state = LieGroup.Identity()
        point = np.random.rand(Tangent.Dim)
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)
        w_point = np.random.rand(Tangent.Dim) * 1e-4

        J_pout_s = np.zeros((LieGroup.Dim, LieGroup.DoF))
        J_pout_p = np.zeros((LieGroup.Dim, LieGroup.Dim))

        pointout = state.act(point, J_pout_s, J_pout_p)

        point_pert = (state + w).act(point)
        point_lin  = pointout + J_pout_s @ w.coeffs()

        assert np.allclose(point_pert, point_lin)

        point_pert = state.act(point + w_point)
        point_lin  = pointout + J_pout_p @ w_point

        assert np.allclose(point_pert, point_lin)

    def test_TanPlusTanJac(self, LieGroup, Tangent):
        delta = Tangent.Random()
        delta_other = Tangent.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)

        J_tout_t0 = np.zeros((LieGroup.DoF, LieGroup.DoF))
        J_tout_t1 = np.zeros((LieGroup.DoF, LieGroup.DoF))

        delta_out = delta.plus(delta_other, J_tout_t0, J_tout_t1)

        delta_pert = (delta+w).plus(delta_other)
        delta_lin  = delta_out.plus(J_tout_t0*w)

        assert delta_pert == delta_lin

        delta_pert = delta.plus(delta_other+w)
        delta_lin  = delta_out.plus(J_tout_t1*w)

        assert delta_pert == delta_lin

    def test_TanMinusTanJac(self, LieGroup, Tangent):
        delta = Tangent.Random()
        delta_other = Tangent.Random()
        w = Tangent(np.random.rand(Tangent.DoF, 1)*1e-4)

        J_tout_t0 = np.zeros((LieGroup.DoF, LieGroup.DoF))
        J_tout_t1 = np.zeros((LieGroup.DoF, LieGroup.DoF))

        delta_out = delta.minus(delta_other, J_tout_t0, J_tout_t1)

        delta_pert = (delta+w).minus(delta_other)
        delta_lin  = delta_out.plus(J_tout_t0*w)

        assert delta_pert == delta_lin

        delta_pert = delta.minus(delta_other+w)
        delta_lin  = delta_out.plus(J_tout_t1*w)

        assert delta_pert == delta_lin
