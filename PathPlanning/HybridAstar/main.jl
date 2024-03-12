using Parameters
using NearestNeighbors
using ProgressMeter
using LinearAlgebra
using Plots

include("src/types.jl")
include("src/setup.jl")
include("src/astar_utils.jl")
include("../ReedsSheppsCurves/src/ReedsSheppsUtils.jl")

obs_list = [[0,-5,0,1,1.5], [-10,-5,0,1.1,1.5],[-5,-7,0,5,1]]

gear_set = [1, -1]
minR = 5.0
steer_set = collect(LinRange(-1/minR,1/minR,7))
expand_time = 1.0
resolutions = [0.2, 0.2, pi/10]
stbound = [-15 15; -15 5; -pi pi]
starting_real = [10.0,-5.0,0.0]
ending_real = [-5.0,-5.0,0.0]
draw_fig = false

hybrid_astar = defineHybridAstar(gear_set, steer_set, minR, expand_time, resolutions, stbound, starting_real, ending_real, draw_fig)
# hybrid_astar = defineHybridAstar(BoundStates, [51, 51], starting_states[1:2], ending_states[1:2], true)
defineHybridAstarobs!(hybrid_astar, obs_list)
@time planHybridAstar!(hybrid_astar)

h = plot(hybrid_astar.r.actualpath[1,:], hybrid_astar.r.actualpath[2,:])
for i in 1:size(hybrid_astar.r.hybrid_astar_states, 2)
    cur_st = hybrid_astar.r.hybrid_astar_states[:, i]
    arrowsize = 0.1
    quiver!(h, [cur_st[1]], [cur_st[2]], quiver = ([arrowsize*cos(cur_st[3])], [arrowsize*sin(cur_st[3])]),color =:red, legend = false)
    display(h)
    sleep(0.1)
end

# h = plotRes(astar)
# display(h)