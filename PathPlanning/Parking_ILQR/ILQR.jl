using Plots
using LinearAlgebra
import Random
using ForwardDiff
# Random.seed!(1234)

include("Cost.jl")
include("Dynamics.jl")
include("GetMatrix.jl")

############# Define initial states and initial controls #############
states = [0;3.6;5;0]
ctrls = [1.0; 0.0]
############# Define Settings #############
N = 30
δT = 0.05
nstates = 4
nctrls = 2
############# Define Data Structures for Storage #############
StatesList = zeros(nstates, N)
StatesList[:, 1] = states
CtrlsList = zeros(nctrls, N)

states_new = zeros(size(StatesList))
control_new = zeros(size(CtrlsList))
states_new[:, 1] = StatesList[:, 1]

klist = zeros( 2, 1, N - 1)
Klist = zeros( 2, 4, N - 1)
############# Roll Out Initial Guess #############
for step = 2:N
    global states, StatesList
    ctrls = [-2.6; 0.01]
    states = RK4Integration(states, ctrls, δT)
    StatesList[:, step] = states
    CtrlsList[:, step - 1] = ctrls
end
## Calculate the Cost of First Roll Out
J = TotalCost(StatesList, CtrlsList)
J_new = J

################# Start iLQR #################
iter = 1
while abs((J_new - J) / J) > 1e-6 || iter == 1
    global iter, klist, Klist, Alpha, StatesList, CtrlsList, states_new, control_new, J, J_new
    Vx, _, Vxx, _, _ = CalculateMatrix(StatesList[:, end], [0 0], TerminalCost)

    J = J_new
#################### Backward Propogation #################### 
    for j in N-1:-1:1
        xc = StatesList[:, j]
        uc = CtrlsList[:, j]
        fx, fu = LocallyLinearizeDynamics(xc, uc, δT)
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

#################### Forward Propogation #################### 
    Alpha = 1
    while J_new >= J
        for i = 1:1:N-1
            u_old = CtrlsList[:, i]
            xtilde = states_new[:, i]
            xn = StatesList[:, i]
            u = u_old + Alpha * klist[:, :, i] + Klist[:, :, i] * (xtilde .- xn)
            control_new[:, i] = u
            states_new[:, i+1] = RK4Integration(xtilde, u, δT)
        end
        J_new = TotalCost(states_new, control_new)
        Alpha = Alpha / 2

        if Alpha <= 1e-3
            break
        end
    end

    StatesList[:,:] = states_new[:,:]
    CtrlsList[:,:] = control_new[:,:]
    iter = iter + 1
    println(iter, "  ", J_new)
end

# plot(StatesList[1,:], StatesList[2,:])


plot(CtrlsList[1,:])