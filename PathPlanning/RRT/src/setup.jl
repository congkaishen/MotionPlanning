
function defineRRT(
    sampling_size::Int = 10000,
    starting_pose = [35.0; 50.0],
    ending_pose = [100.0; 50.0],
    buffer_size = 100,
    BoundPosition = [0; 100; 0; 100],
    rrt_star = false,
    bias_rate = 0.2,
    draw_fig = false,
    make_gif = false,
    )::RRTSearcher
    rrt = RRTSearcher();
    # start node and end node
    start_node = RRTNode(starting_pose, 0.0)
    end_node = RRTNode(ending_pose, Inf)
    rrt.s.buffer_size = buffer_size
    rrt.s.sampling_number = sampling_size
    rrt.s.starting_position = starting_pose
    rrt.s.ending_position = ending_pose
    rrt.s.BoundPosition = BoundPosition

    rrt.p.costs_collection = Vector{Float64}(undef, sampling_size)
    rrt.p.states_collection = Matrix{Float64}(undef, 2, sampling_size)

    rrt.p.starting_node = start_node
    rrt.p.ending_node = end_node
    rrt.p.nodes_collection[rrt.p.sample_idx] = start_node
    rrt.p.costs_collection[rrt.p.sample_idx] = start_node.cost
    rrt.p.states_collection[:,rrt.p.sample_idx] = rrt.p.starting_node.loc
    rrt.p.children_collection[rrt.p.sample_idx] = Set{Int}()
    rrt.p.parents_collection[rrt.p.sample_idx] = -1
    rrt.p.kdtree = KDTree(rrt.p.states_collection[1:2, 1:1])
    rrt.p.balltree = BallTree(rrt.p.states_collection[:, 1:1], MyMetrics())
    rrt.s.rrt_star = rrt_star
    rrt.s.bias_rate = bias_rate
    rrt.s.draw_fig = draw_fig
    rrt.s.make_gif = make_gif
    return rrt
end


function defineRRTobs!(rrt::RRTSearcher, obstacle_list)
    rrt.s.obstacle_list = obstacle_list
    return nothing
end




