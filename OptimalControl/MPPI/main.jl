include("src/MPPIUtils.jl")

using Interpolations
using Plots


X0 = [0, 0, 0, 0, 0, 5, 0]
XL = [-10, -20, -2, -pi/2,  -pi/2, 1, -pi/9]
XU = [130, 20, 2, pi/2,  pi/2, 10, pi/9]
CL = [-0.5, -2.5]
CU = [0.5, 2.5]
numStates = 7
numControls = 2
N = 20
T = 3.0
Δt = T/N

update_time = 0.1
δt = 1e-3
update_idx = Int32(floor(update_time/δt))

SamplingNumber = 2000
goal = [110, 0]
lambda = 25.0
GridNum = [100, 100];

Σ = [0.15 0; 0.0 3];
global NominalControls = ones(N, numControls) .* 0.0
mppi = defineMPPI(numStates, numControls, lambda, X0, goal, SamplingNumber, GridNum, N, T, XL, XU, CL, CU)
defineMPPIobs!(mppi, [[50.0, 1, 2.5], [70.0, -1, 2.5], [90.0, 1, 2.5]])
global time_serial = collect(range(0.0, T, length=N))
global fined_time_serial = collect(range(0.0, update_time, length=update_idx))
global controls = zeros(update_idx, size(NominalControls ,2))
global states = X0
global states_his = [0; states]

time_solve = []

for time_idx = 1:1:Int32(floor(15/δt))
    global NominalControls, time_serial, fined_time_serial, controls, states, states_his
    
    if mod(time_idx - 1, update_idx) + 1 == 1
        ShiftInitialCondition(mppi, states) 
        defineMPPINominalControl!(mppi, NominalControls)
        MPPIPlan(mppi)
        NominalControls = mppi.r.Control
        println("Current time: ", round( time_idx * δt; digits = 1), "  Status: ", mppi.r.Feasibility," (",round(1000 * mppi.r.time; digits = 1)," ms)")

        local interp_linear_u1 = interpolate((time_serial,), NominalControls[:,1], Gridded(Constant{Previous}()))
        local interp_linear_u2 = interpolate((time_serial,), NominalControls[:,2], Gridded(Constant{Previous}()))
        controls[:,:] = [interp_linear_u1(fined_time_serial) interp_linear_u2(fined_time_serial)]
        h = plotRes(mppi, states_his)
        display(h)
        sleep(0.1)
    end
    dstates, _, _ = VehicleDynamics(states, controls[mod(time_idx - 1, update_idx) + 1, :])
    # println(mod(time_idx - 1, update_idx) + 1)
    states = states .+ dstates*δt
    states_his = [states_his [time_idx * δt; states]]
    if (states_his[2, end] - goal[1])^2 + (states_his[3, end] - goal[2])^2 <= 7.2^2
        break
    end
end



