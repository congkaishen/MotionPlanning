
cd(@__DIR__)



include("src/hybrid_astar_utils.jl")

# starting_real = [-15.0, 0.0, 0.0]
# ending_real = [15, 0.0, 0.0]
# block_info = [[0.0, 0.0, 0.0, 5, 5]]

starting_real = [6, 0, pi/2]
ending_real = [0.0,1.0, pi/2]
block_info = [[0.0, -1.0, 0.0, 5.5/2 + 1.0, 1.0], [-5.5/2, 2.7432/2, 0.0, 1.0, 2.7432/2], [5.5/2, 2.7432/2, 0.0, 1.0, 2.7432/2]]
# block_info = [[0.0, -1.0, 0.0, 5.5/2 + 1.0, 1.0], [-5.5/2, 5/2, 0.0, 1.0, 5/2], [5.5/2, 5/2, 0.0, 1.0, 5/2]]
############# The setting here influence discretization ############# 
vehicle_size = [3, 2]
max_δf = pi/6

minR = vehicle_size[1]/tan(max_δf)
gear_set = [1, -1]
steer_set = collect(LinRange(-1/minR,1/minR, 31))
expand_time = 2.5
resolutions = [0.5, 0.5, pi/12]
stbound = [-5 10; 0 10; -pi pi]

use_astar = false

draw_fig = true
make_gif = true
if make_gif
    if isdir("./gifholder")
        println("Already Exists")
        foreach(rm, filter(endswith(".gif"), readdir("./gifholder",join=true)))
    else
        mkdir("./gifholder")
    end
end

hybrid_astar = defineHybridAstar(vehicle_size, gear_set, steer_set, minR, expand_time, resolutions, stbound, starting_real, ending_real, use_astar, draw_fig, make_gif)
defineHybridAstarobs!(hybrid_astar, block_info)
@time planHybridAstar!(hybrid_astar)



