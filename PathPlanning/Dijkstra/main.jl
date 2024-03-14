using Parameters
using NearestNeighbors
using LinearAlgebra
using Plots
using MAT

cd(@__DIR__)

include("src/types.jl")
include("src/setup.jl")
include("src/dka_utils.jl")

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


draw_fig = false
make_gif = false
if make_gif
    if isdir("./gifholder")
        println("Already Exists")
        foreach(rm, filter(endswith(".gif"), readdir("./gifholder",join=true)))
    else
        mkdir("./gifholder")
    end
end

dka = defineDKA(BoundPosition, [51, 51], starting_pose[1:2], ending_pose[1:2], draw_fig, make_gif)
defineDKAobs!(dka, obs_location)
@time planDKA!(dka)
h = plotRes(dka)
display(h)