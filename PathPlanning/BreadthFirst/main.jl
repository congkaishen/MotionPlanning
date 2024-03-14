using Parameters
using NearestNeighbors
using LinearAlgebra
using Plots
using MAT

cd(@__DIR__)

include("src/types.jl")
include("src/setup.jl")
include("src/bfs_utils.jl")


starting_pose = [0.0; 0.0]
ending_pose = [120.0; 120.0]
BoundPosition = [0; 120; 0; 120]


obsinfo = matread("../Scenarios/obstacle_field.mat");
obsinfo = obsinfo["obstacle_field"]
Trial_num = 53
obs_location_temp = obsinfo[Trial_num]
obs_location = Vector{Vector{Float64}}(undef, size(obs_location_temp, 1))
for i = 1:1:size(obs_location_temp, 1)
    obs_location[i] = obs_location_temp[i, :]
end

# starting_ang = [0.0; 0.0]
# starting_ux = 2.0
# ending_pose = [100.0; 100.0]
# ending_ang = [0.0; 0.0]
# ending_ux = 2.0
# buffer_size = 100
# BoundPosition = [0; 100; 0; 100]
# obs_list = [[50,50,25],[15,15,8],[70,10,0],[10,70,10],[90,50,10],[50,90,10]]


bfs = defineBFS(BoundPosition, [51, 51], starting_pose[1:2], ending_pose[1:2])
defineBFSobs!(bfs, obs_location)
planBFS!(bfs)

# plot(bfs.r.actualpath[:,1], bfs.r.actualpath[:,2])