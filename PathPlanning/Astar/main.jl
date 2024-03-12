using Parameters
using NearestNeighbors
using ProgressMeter
using LinearAlgebra
using Plots

include("src/types.jl")
include("src/setup.jl")
include("src/astar_utils.jl")

starting_pose = [0.0; 0.0]
starting_ang = [0.0; 0.0]
starting_ux = 2.0
ending_pose = [100.0; 100.0]
ending_ang = [0.0; 0.0]
ending_ux = 2.0
buffer_size = 100
BoundPosition = [0; 100; 0; 100]
obs_list = [[50,50,25],[15,15,8],[70,10,0],[10,70,10],[90,50,10],[50,90,10]]

# starting_pose = [0.0; 0.0]
# starting_ang = [0.0; 0.0]
# starting_ux = 5.0
# ending_pose = [110.0; 0.0]
# ending_ang = [0.0; 0.0]
# ending_ux = 2.0
# buffer_size = 100
# BoundPosition = [-10; 120; -20; 20]
# # obs_list = [[50,50,25],[15,15,8],[70,10,0],[10,70,10],[90,50,10],[50,90,10]]
# obs_list = [[50.0, 1, 2.5], [70.0, -1, 2.5], [90.0, 1, 2.5]]




astar = defineAstar(BoundPosition, [501, 501], starting_pose[1:2], ending_pose[1:2], false)
defineAstarobs!(astar, obs_list)
@time planAstar!(astar)
# plot(astar.r.actualpath[:,1], astar.r.actualpath[:,2])
# h = plotRes(astar)
# display(h)