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
n = define(numStates=2,numControls=1,X0=[10.,0],XF=[0.,0.],CL=[0.],CU=[3.])
states!(n,[:h,:v];descriptions=["h(t)","v(t)"])
controls!(n,[:T];descriptions=["T(t)"])

dx=[:(v[j]),:(T[j]-1.625)]
dynamics!(n,dx)

configure!(n, N = 30;(:integrationScheme=>:bkwEuler),(:finalTimeDV=>true));
obj = integrate!(n,:(T[j]))
@objective(n.ocp.mdl, Min, obj + 5 * n.ocp.tf);
OCPoptimize!(n)
δt = 0.01
x0 = [10 0 0 0 ]
states = x0
timelist = collect(range(0, n.r.ocp.tctr[end], length=31))


InterpolateT = interpolate((n.r.ocp.tctr ,), n.r.ocp.U[:, 1] .+ (rand(31) .- 0.5) .* 0.7, Gridded(Constant{Next}()))
InterpolateTR = interpolate((n.r.ocp.tctr ,), (n.r.ocp.U[:, 1]), Gridded(Constant{Next}()))


# simulate open loop response
for t_sim = 0:δt:n.r.ocp.tctr[end]
    global states
    x = states[end, 1:2]
    u = InterpolateT(t_sim)
    u = min(max(u, 0), 3)
    uR = InterpolateTR(t_sim)
    dxst = MoonLander(x, u)
    x = x + δt * dxst 
    states = [states; [x' u uR]]
end

plot(states[:, 1])
CSV.write("OCPwNoise.csv",  Tables.table(states), writeheader=false)