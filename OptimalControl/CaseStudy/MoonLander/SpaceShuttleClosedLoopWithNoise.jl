using NLOptCtrl
using JuMP
using Interpolations
import Random
using Plots
using Tables, CSV
Random.seed!(1239)

function MoonLander(x, u)
    dx1 = x[2]
    dx2 = u - 1.625;
    return [dx1, dx2]
end
exec_horizon = 0.1
n = define(numStates=2,numControls=1,X0=[10.,0],XF=[NaN,NaN],XL = [0, -5], XU = [10, 0], CL=[0.],CU=[3.])
states!(n,[:h,:v];descriptions=["h(t)","v(t)"])
controls!(n,[:T];descriptions=["T(t)"])

dx=[:(v[j]),:(T[j]-1.625)]
dynamics!(n,dx)

configure!(n, N = 30;(:integrationScheme=>:bkwEuler),(:tf=>3), (:finalTimeDV=>false));
obj = integrate!(n,:((T[j])^2 + 0.01 * h[j]^2))
h = n.r.ocp.x[:, 1]
v = n.r.ocp.x[:, 2]
@objective(n.ocp.mdl, Min, obj + 200 * h[end]^2 + 120 * v[end]^2);
OCPoptimize!(n)
δt = 0.01
x0 = [10 0 0 0 ]
states = x0


InterpolateT = interpolate((n.r.ocp.tctr ,), (n.r.ocp.U[:, 1] .+ rand(31) * 0.2), Gridded(Constant{Next}()))
InterpolateTR = interpolate((n.r.ocp.tctr ,), (n.r.ocp.U[:, 1]), Gridded(Constant{Next}()))


# simulate closed loop response
for t_sim = 0:δt:6
    global states, InterpolateT, InterpolateTR
    x = states[end, 1:2]
    if abs(floor(t_sim/exec_horizon) - t_sim/exec_horizon) <= 1e-7
        ShiftInitStates(n, x)
        OCPoptimize!(n)
        local timelist
        timelist = collect(range(t_sim, t_sim+ n.r.ocp.tctr[end], length=31))
        InterpolateT = interpolate((timelist ,), (n.r.ocp.U[:, 1] .+ (rand(31) .- 0.5) .* 0.7), Gridded(Constant{Next}()))
        InterpolateTR = interpolate((timelist ,), (n.r.ocp.U[:, 1]), Gridded(Constant{Next}()))
    end
    u = InterpolateT(t_sim)
    uR = InterpolateTR(t_sim)

    u = min(max(u, 0), 3)
    dxst = MoonLander(x, u)
    x = x + δt * dxst 
    states = [states; [x' u uR]]
    if abs(x[1]) < 0.07 && abs(x[2]) < 0.03
        break;
    end 
end


plot(states[:, 1])
CSV.write("MPCwNoise.csv",  Tables.table(states), writeheader=false)