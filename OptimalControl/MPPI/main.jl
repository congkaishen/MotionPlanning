include("types.jl")
include("setup.jl")
include("MPPIUtils.jl")
include("vehicledynamics.jl")
include("map.jl")
using Interpolations
using Plots


X0 = [0, 0, 0, 0, 0, 3, 0]
XL = [-10, -60, -2, -pi/2,  -pi/2, 1, -pi/9]
XU = [110, 60, 2, pi/2,  pi/2, 6, pi/9]
CL = [-0.5, -2.5]
CU = [0.5, 2.5]
numStates = 7
numControls = 2
N = 15
T = 3.0
Δt = T/N

update_time = 0.2
δt = 1e-3
update_idx = Int32(floor(update_time/δt))

SamplingNumber = 700
goal = [100, 0]
lambda = 25.0
GridNum = [100, 100];

Σ = [0.15 0; 0.0 0.1];
global NominalControls = ones(N, numControls) .* 0.0
mppi = defineMPPI(numStates, numControls, lambda, X0, goal, SamplingNumber, GridNum, N, T, XL, XU, CL, CU, false)
defineMPPIobs!(mppi, [[35.0, -1, 3.0], [50.0, 1, 3.0]])
InitialBlindSplotDetection(mppi)
UpdateTouchVisibleandObsMap(mppi, X0)
global time_serial = collect(range(0.0, T, length=N))
global fined_time_serial = collect(range(0.0, update_time, length=update_idx))
global controls = zeros(update_idx, size(NominalControls ,2))

global states = X0
global states_his = states

time_solve = []

for time_idx = 1:1:Int32(floor(15/δt))
    global NominalControls, time_serial, fined_time_serial, controls, states, states_his, time_solve
    
    if mod(time_idx - 1, update_idx) + 1 == 1
        UpdateTouchVisibleandObsMap(mppi, states)
        ShiftInitialCondition(mppi, states) 
        defineMPPINominalControl!(mppi, NominalControls)
        MPPIPlan(mppi)
        NominalControls = mppi.r.Control
        println("Current time: ", round( time_idx * δt; digits = 1), "  Status: ", mppi.r.Feasibility," (",round(1000 * mppi.r.time; digits = 1)," ms)")
        time_solve = [time_solve; mppi.r.time]
        local interp_linear_u1 = interpolate((time_serial,), NominalControls[:,1], Gridded(Constant{Previous}()))
        local interp_linear_u2 = interpolate((time_serial,), NominalControls[:,2], Gridded(Constant{Previous}()))
        controls[:,:] = [interp_linear_u1(fined_time_serial) interp_linear_u2(fined_time_serial)]
        h = plotRes_withColorMap(mppi, states_his)
        filename = "Results/NoProjection/" * string(time_idx) * ".png"

        savefig(h, filename)
        display(h)
    end
    dstates, _, _ = VehicleDynamics(states, controls[mod(time_idx - 1, update_idx) + 1, :])
    states = states .+ dstates*δt
    states_his = [states_his states]
end

# gif(anim, "WithProjection.gif", fps = 5)




