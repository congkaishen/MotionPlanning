using ForwardDiff
function Dynamics(states, control)
    la      = 1.56
    lb      = 1.64
    x       = states[1]
    y       = states[2]
    ux      = states[3]
    ψ       = states[4]
    ax      = control[1]
    δ       = control[2]
    β       = atan(la / (la + lb) * tan(δ))
    dx      = ux * cos(ψ + β)
    dy      = ux * sin(ψ + β)
    dψ      = ux * cos(β) * tan(δ) / (la + lb)
    dux     = ax
    return [dx dy dux dψ]
end

function RK4Integration(states, control, δT)
    k1      = Dynamics(states, control)
    x2      = states + δT / 2 * k1
    k2      = Dynamics(x2, control)
    x3      = states + δT / 2 * k2;
    k3      = Dynamics(x3, control)
    x4      = states + δT * k3
    k4      = Dynamics(x4, control)
    statesN = 1 / 6 * (k1 + 2 * k2 + 2 * k3 + k4) * δT + states
    return statesN
end

function LocallyLinearizeDynamics(states, control, δT)
    Ak      = ForwardDiff.jacobian((x) -> RK4Integration(x, control, δT), states)
    Bk      = ForwardDiff.jacobian((u) -> RK4Integration(states, u, δT), control)
    return Ak, Bk
end

# function LocallyLinearizeDynamics(states, ctrls, ΔT)
#     ϵ = 1e-3
#     A = zeros(size(states, 2),size(states, 2))
#     B = zeros(size(states, 2),size(ctrls,2))
#     for i in 1:size(states, 2)
#         Δstates = zeros(size(states))
#         Δstates[i] = ϵ
#         f_plus =  RK4Integration(states .+ Δstates, ctrls, ΔT)'
#         f_minus = RK4Integration(states .- Δstates, ctrls, ΔT)'

#         A[:, i] = (f_plus - f_minus)./(2*ϵ)
#     end

#     for j in 1:size(ctrls, 2)
#         Δctrls = zeros(size(ctrls))
#         Δctrls[j] = ϵ
#         f_plus =  RK4Integration(states, ctrls .+ Δctrls, ΔT)'
#         f_minus = RK4Integration(states, ctrls .- Δctrls, ΔT)'
#         B[:, j] = (f_plus - f_minus)./(2*ϵ)
#     end
#     return A, B
# end