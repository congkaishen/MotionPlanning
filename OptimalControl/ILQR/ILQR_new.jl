using Plots
using LinearAlgebra
import Random
Random.seed!(1234)

include("Cost.jl")
include("Dynamics.jl")
include("GetMatrix.jl")

## First let's rollout some trajectories

states = [0 3.6 5 0]
StatesList = copy(states)
Random.seed!(1234)
N = 100
δT = 0.05
ax = Vector{Float64}(undef, N - 1)
δf = Vector{Float64}(undef, N - 1)
ax[1 : N - 1] .= -2.6
δf[1 : N - 1] .= 0.01

for step = 2:N
    global states, StatesList
    states = RK4Integration(states, [ax[step - 1] δf[step - 1]], δT)
    StatesList = [StatesList; states]
end

## Calculate the Cost
ControlList = [ax δf]
J = TotalCost(StatesList, ControlList)
J_new = J

Iteration = 1

#  For the first Iteration
klist = zeros( 2, 1, N - 1)
Klist = zeros( 2, 4, N - 1)
iter = 1
while abs((J_new - J) / J) > 1e-6 || iter == 1
    global iter, Vx, Vxx, StatesList, ControlList, fx, fu, klist, Klist, α, iteration, J_new, StateList, ControlList, J, states_new, control_new
    Vx, _, Vxx, _, _ = CalculateMatrix(StatesList[end, :], [0 0], TerminalCost)

    J = J_new

    for j in N-1:-1:1
        xc = StatesList[j, :]
        uc = ControlList[j, :]
        fx, fu = LocallyLinearizeDynamics(xc', uc', δT)
        lx, lu, lxx, luu, lux = CalculateMatrix(xc, uc, StageCost)


        Qx = lx + fx' * Vx
        Qu = lu + fu' * Vx
        Qxx = lxx + fx' * Vxx * fx
        Quu = luu + fu' * Vxx * fu
        Qux = lux + fu' * Vxx * fx
        k = - pinv(Quu) * Qu
        klist[:, :, j] = k
        K = -pinv(Quu) * Qux
        Klist[:, :, j] = K
        Vx = Qx - K' * Quu * k
        Vxx = Qxx - K' * Quu * K;
    end



    #  Forward pass
    x0 = StatesList[1, :]
    states_new = zeros(size(StatesList))
    states_new[1, :] = x0
    control_new = zeros(size(ControlList))
    J_new = J
    α = 1
    iteration = 1
    while J_new >= J
        for i = 1:1:N-1
            u_old = ControlList[i, :]
            xtilde = states_new[i, :]
            xn = StatesList[i, :]
            u = u_old + α * klist[:, :, i] + Klist[:, :, i] * (xtilde .- xn)
            control_new[i, :] = u
            states_new[i + 1, :] = RK4Integration(xtilde', u', δT)
        end
        J_new = TotalCost(states_new, control_new)
        println(α, "  ", J_new)
        α = α / 2
        iteration = iteration + 1
    end

    StatesList = states_new
    ControlList = control_new
    iter = iter + 1
end

# plot(StatesList[:,1], StatesList[:,2])