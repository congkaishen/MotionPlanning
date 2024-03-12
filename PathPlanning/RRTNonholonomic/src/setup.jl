
function defineRRT(
    sampling_size::Int = 10000,
    starting_pose = [35.0; 50.0],
    starting_ang = 0.0,
    ending_pose = [100.0; 50.0],
    ending_ang = [0.0; 0.0],
    buffer_size = 100,
    BoundPosition = [0; 100; 0; 100],
    rrt_star = false,
    bias_rate = 0.2
    )::RRTSearcher
    rrt = RRTSearcher();
    # start node and end node
    start_node = RRTNode(starting_pose, starting_ang, 0.0)
    end_node = RRTNode(ending_pose, ending_ang, Inf)
    rrt.s.buffer_size = buffer_size
    rrt.s.sampling_number = sampling_size
    rrt.s.starting_position = starting_pose
    rrt.s.ending_position = ending_pose
    rrt.s.BoundPosition = BoundPosition

    rrt.p.costs_collection = Vector{Float64}(undef, sampling_size)
    rrt.p.states_collection = Matrix{Float64}(undef, 3, sampling_size)

    rrt.p.starting_node = start_node
    rrt.p.ending_node = end_node
    rrt.p.nodes_collection[rrt.p.sample_idx] = start_node
    rrt.p.costs_collection[rrt.p.sample_idx] = start_node.cost
    rrt.p.states_collection[:,rrt.p.sample_idx] = [rrt.p.starting_node.loc; rrt.p.starting_node.eulerang]
    rrt.p.children_collection[rrt.p.sample_idx] = Set{Int}()
    rrt.p.parents_collection[rrt.p.sample_idx] = -1
    rrt.p.kdtree = KDTree(rrt.p.states_collection[1:3, 1:1])
    rrt.p.balltree = BallTree(rrt.p.states_collection[:, 1:1], MyMetrics())
    rrt.s.rrt_star = rrt_star
    rrt.s.bias_rate = bias_rate
    return rrt
end


function defineRRTobs!(rrt::RRTSearcher, obstacle_list)
    rrt.s.obstacle_list = obstacle_list
    return nothing
end


function defineRRTterrain!(rrt::RRTSearcher, terrain_kdtree::KDTree, pointcloud::Matrix{Float64})
    rrt.s.terrain_info = pointcloud
    rrt.s.terrain_kdtree = terrain_kdtree
    idxs, dists = knn(rrt.s.terrain_kdtree, rrt.p.starting_node.loc[1:2], 5, true)
    rrt.p.starting_node.loc[3] = rrt.s.terrain_info[3, idxs[1]]

    p_list =  rrt.s.terrain_info[:,idxs]
    cov_matrix = cov(p_list, dims=2)
    vecs = eigvecs(cov_matrix)
    normalvec = vecs[:,1]
    normalvec = normalvec*sign(normalvec[3])
    θ = asin(normalvec[1])
    φ = asin(-normalvec[2]/(cos(θ)))
    ψ = rrt.p.starting_node.eulerang[3]
    rrt.p.starting_node.eulerang = [φ;θ;ψ]

    rrt.p.nodes_collection[1] = rrt.p.starting_node
    rrt.s.starting_position[3] = rrt.p.starting_node.loc[3]

    end_idxs, end_dists = knn(rrt.s.terrain_kdtree, rrt.p.ending_node.loc[1:2], 5, true)
    rrt.p.ending_node.loc[3] = rrt.s.terrain_info[3, end_idxs[1]]
    rrt.s.ending_position[3] = rrt.p.ending_node.loc[3]
    rrt.p.states_collection[:,1] = [rrt.p.starting_node.loc; rrt.p.starting_node.eulerang[3]; rrt.p.starting_node.ux]
    # rrt.p.ending_node
    return nothing
end

