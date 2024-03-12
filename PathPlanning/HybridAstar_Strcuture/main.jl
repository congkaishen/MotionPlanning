using Parameters
using NearestNeighbors
using ProgressMeter
using LinearAlgebra
using Plots

include("src/types.jl")
include("src/setup.jl")
include("src/astar_utils.jl")

starting_states = [0.0; 0.0]
ending_states = [100.0; 100.0]
BoundStates = [0; 100; 0; 100]
obs_list = [[50,50,25],[15,15,8],[70,10,0],[10,70,10],[90,50,10],[50,90,10]]


hybrid_astar = defineHybridAstar(BoundStates, [51, 51], starting_states[1:2], ending_states[1:2], true)
defineHybridAstarobs!(hybrid_astar, obs_list)
@time planHybridAstar!(hybrid_astar)
# plot(astar.r.actualpath[:,1], astar.r.actualpath[:,2])
# h = plotRes(astar)
# display(h)