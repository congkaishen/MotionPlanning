include("src/astar_utils.jl")

cd(@__DIR__)
scenario_num = 2 #1: parking scenario, 2: obstacle field scenario
if scenario_num == 1
    starting_pose = [5, 0]
    ending_pose = [0.0,1.0]
    BoundPosition = [-5;10; 0;10;]
    # obs_location = [[0.0, -1.0, 1.0], [-5.5/2, 2.7432/2, 1], [5.5/2, 2.7432/2, 1]]
    obs_location = [[0.0, -1.0, 0.0, 5.5/2 + 1.0, 1.0], [-5.5/2, 2.7432/2, 0.0, 1.0, 2.7432/2], [5.5/2, 2.7432/2, 0.0, 1.0, 2.7432/2]]
elseif scenario_num == 2
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

astar = defineAstar(BoundPosition, [51, 51], starting_pose[1:2], ending_pose[1:2], draw_fig, make_gif)
defineAstarobs!(astar, obs_location)
@time planAstar!(astar)
println("Path length: $(round(astar.r.path_length, digits=3)) m")

################################# Show the Result ######################################
h = plotRes(astar)
display(h)