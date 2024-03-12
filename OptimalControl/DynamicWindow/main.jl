include("src/DWAUtils.jl")

using Interpolations
using Plots


X0 = [0, 0, 0, 0, 0, 5, 0]
XL = [-10, -20, -2, -pi/2,  -pi/2, 1, -pi/9]
XU = [130, 20, 2, pi/2,  pi/2, 10, pi/9]
CL = [-0.3, -2.5]
CU = [0.3, 2.5]
numStates = 7
numControls = 2
T = 3.0

update_time = 0.1
δt = 1e-3
update_idx = Int32(floor(update_time/δt))

SamplingNumber = 2000
goal = [110, 0]
lambda = 25.0
GridNum = [100, 100];

SampleNumber = [31, 41]
dwa = defineDWA(numStates, numControls, X0, goal, SampleNumber, T, XL, XU, CL, CU)
defineDWAobs!(dwa, [[50.0, 1, 2.5], [70.0, -1, 2.5], [90.0, 1, 2.5]])
global time_serial = collect(range(0.0, T, length=dwa.s.N))
global fined_time_serial = collect(range(0.0, update_time, length=update_idx))
global states = X0
global states_his = [0; states]

time_solve = []

for time_idx = 1:1:Int32(floor(15/δt))
    global NominalControls, time_serial, fined_time_serial, controls, states, states_his
    
    if mod(time_idx - 1, update_idx) + 1 == 1
        ShiftInitialCondition(dwa, states) 
        DWAPlan(dwa)
        println("Current time: ", round( time_idx * δt; digits = 1), "  Status: ", dwa.r.Feasibility," (",round(1000 * dwa.r.time; digits = 1)," ms)")

        controls = dwa.r.Control
        h = plotRes(dwa, states_his)
        display(h)
        sleep(0.1)
    end
    dstates, _, _ = VehicleDynamics(states, controls)
    states = states .+ dstates*δt
    states_his = [states_his [time_idx * δt; states]]
    if (states_his[2, end] - goal[1])^2 + (states_his[3, end] - goal[2])^2 <= 7.2^2
        break
    end
end



