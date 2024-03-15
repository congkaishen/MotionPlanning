function plotTree(h, rrt::RRTSearcher)
    tree = getTree(rrt)
    if tree != nothing
        for i in 1:2:size(tree, 2)
            h = plot!(h, tree[1, i:i+1], tree[2, i:i+1],color=:green, legend=false)
        end
    end
    return h
end

function ellipseShape(x, y, a, b)
	theta = LinRange(0, 2*pi,100)
	x .+ a*cos.(theta), y .+ b*sin.(theta)
end

function plotEnv(rrt::RRTSearcher)
    obs_info = rrt.s.obstacle_list
    obs_num = size(obs_info,1)
    theta = 0:0.01:2*pi
    # plot()
    for obs_idx in 1:obs_num
        ox = obs_info[obs_idx][1]
        oy = obs_info[obs_idx][2]
        r = obs_info[obs_idx][3]
        h = plot!(ellipseShape(ox,oy,r,r),
        seriestype = [:shape],
        lw = 0.5,
        c = :grey,
        linecolor = :black,
        fillalpha = 0.7,
        xlabel = "X[m]",
        ylabel = "Y[m]",
        title = string("RRT result")
        )
    end
    return current()
end

function circleShape(h,k,r)
    θ = LinRange(0, 2*π, 500)
    h.+r*sin.(θ), k.+r*cos.(θ)
end

function plotRes(rrt::RRTSearcher)
    title_string = "Iterations: $(rrt.p.loop_count), Number of Nodes: $(length(rrt.p.nodes_collection))"

    obs_setting = rrt.s.obstacle_list
    h = plot(size = [600, 600])

    for obs_idx = 1:1:size(obs_setting, 1)
        h = plot!(h, circleShape(obs_setting[obs_idx][1], obs_setting[obs_idx][2], obs_setting[obs_idx][3]), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 1.0)
    end

    h = plotTree(h, rrt)

    h = plot!(h, circleShape(rrt.s.ending_position[1], rrt.s.ending_position[2], 3), seriestype = [:shape,], ;w = 0.5, c=:green, linecolor = :green, legend = false, fillalpha = 1.0)
    h = plot!(h, circleShape(rrt.s.starting_position[1], rrt.s.starting_position[2], 1), seriestype = [:shape,], ;w = 0.5, c=:red, linecolor = :red, legend = false, fillalpha = 1.0, title = title_string, xlim=(rrt.s.BoundPosition[1]-2, rrt.s.BoundPosition[2]+2), ylim=(rrt.s.BoundPosition[3]-2, rrt.s.BoundPosition[4]+2))

    if size(rrt.r.Path, 2) > 1
        h = plot!(rrt.r.Path[1,:], rrt.r.Path[2,:], aspect_ratio=:equal, lc=:red, linewidth = 5)
    end
    xlabel!("X [m]")
    ylabel!("Y [m]")

    return h
end

