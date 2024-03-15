include("src/rrt_utils.jl")
cd(@__DIR__)


sampleNum = 30000
rrt_star = true
bias_rate = 0.1
ending_ang = 0.0

starting_pose = [0.0; 0.0]
starting_ang = 0.0
ending_pose = [120.0; 120.0]
ending_ang = 0.0
buffer_size = 100
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

rrt = defineRRT(sampleNum, starting_pose, ending_pose, buffer_size, BoundPosition, rrt_star, bias_rate, draw_fig, make_gif)
defineRRTobs!(rrt, obs_location)
planRRT!(rrt)
plotRes(rrt)
