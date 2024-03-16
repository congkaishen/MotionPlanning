using Parameters
using NearestNeighbors
using ProgressMeter
using LinearAlgebra
using Plots

cd(@__DIR__)


include("src/types.jl")
include("src/setup.jl")
include("src/hybrid_astar_utils.jl")
include("../ReedsSheppsCurves/src/ReedsSheppsUtils.jl")

# starting_real = [-15.0, 0.0, 0.0]
# ending_real = [15, 0.0, 0.0]
# block_info = [[0.0, 0.0, 0.0, 5, 5]]


starting_real = [5, 0, pi/2]
ending_real = [0.0,1.0, -pi/2]
block_info = [[0.0, -1.0, 0.0, 5.5/2 + 1.0, 1.0], [-5.5/2, 2.7432/2, 0.0, 1.0, 2.7432/2], [5.5/2, 2.7432/2, 0.0, 1.0, 2.7432/2]]


gear_set = [1, -1]
minR = 4.0
steer_set = collect(LinRange(-1/minR,1/minR, 5))
expand_time = 2
resolutions = [0.2, 0.2, pi/8]
stbound = [-15 15; -15 15; -pi pi]





draw_fig = false

hybrid_astar = defineHybridAstar(gear_set, steer_set, minR, expand_time, resolutions, stbound, starting_real, ending_real, draw_fig)
# hybrid_astar = defineHybridAstar(BoundStates, [51, 51], starting_states[1:2], ending_states[1:2], true)
defineHybridAstarobs!(hybrid_astar, block_info)
@time planHybridAstar!(hybrid_astar)


# h = plot(hybrid_astar.r.actualpath[1,:], hybrid_astar.r.actualpath[2,:])
# h = PlotWall(h, Block2Pts(hybrid_astar.s.obstacle_list))
# for i in 1:size(hybrid_astar.r.hybrid_astar_states, 2)
#     cur_st = hybrid_astar.r.hybrid_astar_states[:, i]
#     arrowsize = 0.1
#     quiver!(h, [cur_st[1]], [cur_st[2]], quiver = ([arrowsize*cos(cur_st[3])], [arrowsize*sin(cur_st[3])]),color =:red, legend = false)
#     display(h)
#     sleep(0.1)
# end
h = plotRes(hybrid_astar)
display(h)

# h = plotRes(astar)
# display(h)