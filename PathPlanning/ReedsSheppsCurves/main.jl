include("./src/ReedsSheppsUtils.jl")

x0 = 10*(2*rand(1)[1]-1)
y0 = 10*(2*rand(1)[1]-1)
ψ0 = pi*(2*rand(1)[1]-1)
init_st = [x0, y0, ψ0]


xg = 10*(2*rand(1)[1]-1)
yg = 10*(2*rand(1)[1]-1)
ψg = pi*(2*rand(1)[1]-1)

termi_st = [xg, yg, ψg]
minR = 10*(2*rand(1)[1]-1)
norm_states = changeBasis(init_st, termi_st, minR)
opt_cmd, opt_cost, cmds, costs = allpath(norm_states)
act_opt_cost = opt_cost*minR
    

using Plots
h1 = plotpaths(cmds, costs)

h2 = plotActpaths(init_st, minR, cmds, costs)

println(init_st)
println(termi_st)
display(h1)