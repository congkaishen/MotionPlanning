include("src/rrt_utils.jl")
cd(@__DIR__)


sampleNum = 10000
rrt_star = false
bias_rate = 0.05
ending_ang = 0.0

# starting_pose = [0.0; 0.0]
# starting_ang = 0.0
# ending_pose = [110.0; 0.0]
# buffer_size = 100
# BoundPosition = [-10; 120; -10; 10]
# obs_location = [[50.0, 1, 2.5], [70.0, -1, 2.5], [90.0, 1, 2.5]]

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



rrt = defineRRT(sampleNum, starting_pose, ending_pose, buffer_size, BoundPosition, rrt_star, bias_rate )
defineRRTobs!(rrt, obs_location)
planRRT!(rrt)
plotRes(rrt)














# defineRRTcandidate(tpp.rrt)
# tpp.rrt.s.buffer_size = buffer_size
# tpp.rrt.s.draw_fig = false

# defineLTRKeyParams(tpp.ltr, 1, 0.2, 0.1, 0.02)
# # println("Start Planning")
# planAstar!(tpp.astar)
# total_length = sum( sqrt.( (tpp.astar.r.actualpath[1:end-1, 1] - tpp.astar.r.actualpath[2:end, 1]).^2 + (tpp.astar.r.actualpath[1:end-1, 2] - tpp.astar.r.actualpath[2:end, 2]).^2) )
# gap =  Int64(  maximum([floor(size(tpp.astar.r.actualpath, 1)/(floor(total_length/tpp.rrt.s.sample_max_dist))),  1])  )
# sampling_bias_points = tpp.astar.r.actualpath[1:gap:end, :]

# tpp.rrt.p.sampling_bias_points = sampling_bias_points
# planRRT!(tpp.rrt)

# plotRes(tpp.rrt)
