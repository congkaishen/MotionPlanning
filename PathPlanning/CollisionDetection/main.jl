include("src/utils.jl")



cd(@__DIR__)
using Plots
block_info = [[0.0, -1.0, 0.0, 5.5/2 + 1.0, 1.0], [-5.5/2, 2.7432/2, 0.0, 1.0, 2.7432/2], [5.5/2, 2.7432/2, 0.0, 1.0, 2.7432/2]]
veh_block = [5.5, 1.0, 0, 3/2, 2/2]

pts = Block2Pts(block_info)
veh_pts = GetRectanglePts(veh_block)
println(ConvexCollision(pts[:,:,1], veh_pts))



cur_pt = [0.0;-1.0]
epsilon =1e-1
mypts = [[cur_pt[1];cur_pt[2]+epsilon] [cur_pt[1]-epsilon;cur_pt[2]] [cur_pt[1]+epsilon;cur_pt[2]] [cur_pt[1];cur_pt[2]+epsilon]]
ConvexCollision(pts[:,:,1], mypts)

h = plot()
h = PlotWall(h, pts)
h = PlotVehicle(h, veh_pts)
h = plot!(h, mypts[1,:], mypts[2,:], color = :green, linestyle=:dash)
display(h)