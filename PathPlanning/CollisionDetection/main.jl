include("src/utils.jl")



cd(@__DIR__)
using Plots
block_info = [[0.0, -1.0, 0.0, 5.5/2 + 1.0, 1.0], [-5.5/2, 2.7432/2, 0.0, 1.0, 2.7432/2], [5.5/2, 2.7432/2, 0.0, 1.0, 2.7432/2]]
veh_block = [5.5, 1.0, 0, 3/2, 2/2]

pts = Block2Pts(block_info)
veh_pts = GetRectanglePts(veh_block)

println(ConvexCollision(pts[:,:,1], veh_pts))

h = plot()
h = PlotWall(h, pts)
h = PlotVehicle(h, veh_pts)
display(h)