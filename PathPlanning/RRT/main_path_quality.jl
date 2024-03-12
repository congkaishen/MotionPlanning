include("src/tpp_utils.jl")

# Trial_num = readdlm("Trial_num.txt", '\t', Int, '\n')
# Trial_num = Trial_num[1]

# print("Random Trial Idx: ")
# println(Trial_num)

sampleNum = 30000
use_astar = true
rrt_star = false
single = false
success_case = 0
bias_rate = [0.1, 0.65]

if !use_astar
    bias_rate[2] = bias_rate[1]
end

starting_pose = [0.0; 0.0; pi/4]
starting_ang = [0.0; 0.0; 0.0]
starting_ux = 1.0
ending_pose = [120.0; 120.0; 0.0]
ending_ang = [0.0; 0.0; 0.0]
ending_ux = 1.0
buffer_size = 100
BoundPosition = [0; 120; 0; 120]

obsinfo = matread("obstacle_field.mat");
obsinfo = obsinfo["obstacle_field"]
Trial_num = 53
obs_location_temp = obsinfo[Trial_num]
obs_location = Vector{Vector{Float64}}(undef, size(obs_location_temp, 1))
for i = 1:1:size(obs_location_temp, 1)
    obs_location[i] = obs_location_temp[i, :]
end

xNum = 200
yNum = 200
Xterrain = range(0, 100, xNum)
Yterrain = range(0, 100, yNum)
f(x, y) = begin
    5*sin(x/9+pi/2)*cos(y/15)  # (3x + y ^ 2) * abs(sin(x) + cos(y))
end 


xNum = size(Xterrain, 1)
yNum = size(Yterrain, 1)
global pointcloud = zeros(3,xNum*yNum)
global idx = 1
for x in Xterrain
    for y in Yterrain
        global pointcloud
        global idx
        local z = f(x,y)
        pointcloud[:,idx] = [x;y;z]
        idx = idx+1
    end
end
XMAP = repeat(reshape(Xterrain, 1, :), length(Yterrain), 1)
YMAP = repeat(Yterrain, 1, length(Xterrain))
ZMAP = map(f, XMAP, YMAP)



plot()
contour!(Xterrain, Yterrain, ZMAP, fill = true, levels = minimum(ZMAP):((maximum(ZMAP)-minimum(ZMAP))/10.0):maximum(ZMAP), legend = false)


tpp = defineTPP(sampleNum,starting_pose,starting_ang,starting_ux,ending_pose,ending_ang,ending_ux,buffer_size,BoundPosition, rrt_star, bias_rate)
defineTPPobs!(tpp, obs_location)
defineRRTcandidate(tpp.rrt)
tpp.rrt.s.buffer_size = buffer_size
tpp.rrt.s.draw_fig = false

defineTPPterrain!(tpp, terrain_kdtree, pointcloud)
defineLTRKeyParams(tpp.ltr, 150, 0.2, 0.1, 0.02)
# println("Start Planning")
planTPP(tpp)
rrt_tree = retrieveTree(tpp.rrt)
tpp_result = [tpp.astar.r; tpp.rrt.r;tpp.ltr.r;[rrt_tree]]
file = matopen("tpp_result.mat", "w")
write(file, "tpp_result", tpp_result )
close(file)
