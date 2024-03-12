function defineTPP(sampling_size::Int = 10000,
    starting_pose = [35.0; 50.0; 0.0],
    starting_ang = [0.0; 0.0; 0],
    starting_ux = 2.0,
    ending_pose = [100.0; 50.0; 0.0],
    ending_ang = [0.0; 0.0; 0.0],
    ending_ux = 2.0,
    buffer_size = 100,
    BoundPosition = [0; 100; 0; 100],
    rrt_star = false,
    bias_rate = [0.2, 0.7])::TPPSearcher
    tpp = TPPSearcher()
    tpp.rrt = defineRRT(sampling_size, starting_pose, starting_ang, starting_ux, ending_pose, ending_ang, ending_ux, buffer_size, BoundPosition, rrt_star, bias_rate)
    tpp.ltr = defineLTR()
    tpp.astar = defineAstar(BoundPosition, [51, 51], starting_pose[1:2], ending_pose[1:2])

    return tpp
end



function defineTPPobs!(tpp::TPPSearcher, obstacle_list)
    obstacle_list = Vector{Float64}.(obstacle_list)
    defineRRTobs!(tpp.rrt, obstacle_list)
    defineLTRobs!(tpp.ltr, obstacle_list)
    if size(obstacle_list, 1) != 0
        for i = 1:1:size(obstacle_list, 1)
            obstacle_list[i][3] = obstacle_list[i][3] * 1.1
        end
    end
    defineAstarobs!(tpp.astar, obstacle_list)
    return nothing
end


function defineTPPterrain!(tpp::TPPSearcher, terrain_kdtree::KDTree, pointcloud::Matrix{Float64})
    defineRRTterrain!(tpp.rrt, terrain_kdtree, pointcloud)
    defineLTRterrain!(tpp.ltr, terrain_kdtree, pointcloud)
    defineAstarTerrain!(tpp.astar, terrain_kdtree, pointcloud)
    return nothing
end




################################################################################   TPP Setup Ends  #########################################################################################



################################################################################## Astar Setup ##########################################################################################

function defineAstar(
    realsize = [-50; 0; 50; 100],
    mapbound = [51; 51],
    starting_real = [-3; 20],
    ending_real = [50, 20],
    )::AstarSearcher
    astar = AstarSearcher();
    astar.s.actualbound = realsize
    astar.s.mapbound = mapbound
    astar.s.starting_real = starting_real
    astar.s.ending_real = ending_real
    astar.s.x_factor = (astar.s.actualbound[2] - astar.s.actualbound[1])/(astar.s.mapbound[1] - 1)
    astar.s.y_factor = (astar.s.actualbound[4] - astar.s.actualbound[3])/(astar.s.mapbound[2] - 1)
    astar.s.starting_pos = [Int(floor((astar.s.starting_real[1] - astar.s.actualbound[1])/astar.s.x_factor + 1)), Int(floor((astar.s.starting_real[2] - astar.s.actualbound[3])/astar.s.y_factor + 1))]
    astar.s.ending_pos = [Int(floor((astar.s.ending_real[1] - astar.s.actualbound[1])/astar.s.x_factor + 1)), Int(floor((astar.s.ending_real[2] - astar.s.actualbound[3])/astar.s.y_factor + 1))]
    astar.p.starting_node = AstarNode(nothing, astar.s.starting_pos, 0.0, 0, 0, 0)

    astar.p.nodes_collection[Encode(astar, astar.s.starting_pos)] = astar.p.starting_node
    return astar
end


function defineAstarobs!(astar::AstarSearcher, obstacle_list)
    astar.s.obstacle_list = obstacle_list
    return nothing
end


function defineAstarTerrain!(astar::AstarSearcher, terrain_kdtree::KDTree, pointcloud::Matrix{Float64})
    astar.s.terrain_info = pointcloud
    astar.s.terrain_kdtree = terrain_kdtree
    idxs_start, ~ = knn(terrain_kdtree, astar.s.starting_real[1:2], 5, true)
    astar.p.starting_node.height = astar.s.terrain_info[3, idxs_start[1]];
end


###################################################################################   RRT Setup  #########################################################################################


###################################################################################   RRT Setup  #########################################################################################


function defineRRT(
    sampling_size::Int = 10000,
    starting_pose = [35.0; 50.0; 0.0],
    starting_ang = [0.0; 0.0; 0],
    starting_ux = 2.0,
    ending_pose = [100.0; 50.0; 0.0],
    ending_ang = [0.0; 0.0; 0.0],
    ending_ux = 2.0,
    buffer_size = 100,
    BoundPosition = [0; 100; 0; 100],
    rrt_star = false,
    bias_rate = [0.2, 0.7]
    )::RRTSearcher
    rrt = RRTSearcher();
    # start node and end node
    start_node = TPPNode(starting_pose, starting_ang, starting_ux, 0.0)
    end_node = TPPNode(ending_pose, ending_ang, ending_ux, Inf)
    rrt.s.buffer_size = buffer_size
    rrt.s.sampling_number = sampling_size
    rrt.s.starting_position = starting_pose
    rrt.s.ending_position = ending_pose
    rrt.s.BoundPosition = BoundPosition

    rrt.p.costs_collection = Vector{Float64}(undef, sampling_size)
    rrt.p.states_collection = Matrix{Float64}(undef, 5, sampling_size)

    rrt.p.starting_node = start_node
    rrt.p.ending_node = end_node
    rrt.p.nodes_collection[rrt.p.sample_idx] = start_node
    rrt.p.costs_collection[rrt.p.sample_idx] = start_node.cost
    rrt.p.states_collection[:,rrt.p.sample_idx] = [rrt.p.starting_node.loc; rrt.p.starting_node.eulerang[3]; rrt.p.starting_node.ux]
    rrt.p.children_collection[rrt.p.sample_idx] = Set{Int}()
    rrt.p.parents_collection[rrt.p.sample_idx] = -1
    Classifier = matread("src/ClassifierInfo.mat");
    LayerInfo = Classifier["LayerInfo"]
    rrt.s.LayerInfo = LayerInfo
    rrt.p.kdtree = KDTree(rrt.p.states_collection[1:3, 1:1])
    rrt.p.balltree =BallTree(rrt.p.states_collection[:, 1:1], MyMetrics())
    rrt.s.rrt_star = rrt_star
    rrt.s.bias_rate = bias_rate
    return rrt
end

function defineRRTcandidate(rrt)
    candidate_points = matread("src/candidate_point.mat");
    candidacy_all = candidate_points["candidate_point"]

    rrt.s.candidacy = transpose(candidacy_all)
    rrt.s.candidacytree =BallTree(rrt.s.candidacy, CandidacyMetrics())
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


################################################################################   TPP Setup Ends  #########################################################################################


###################################################################################   LTR Setup  #########################################################################################
function defineLTR()::LTRSearcher
    ltr = LTRSearcher();
    # start node and end node
    return ltr
end



function PortingLTRTraj!(ltr::LTRSearcher, BoundPosition = [0; 100; 0; 100])
    traj = ltr.p.traj
    start_node = TPPNode(traj[1, 1:3], traj[1, 4:6], traj[1, 7], traj[1, 8])
    end_node = TPPNode(traj[end, 1:3], traj[end, 4:6], traj[end, 7], traj[end, 8])

    ltr.s.sampling_number = size(traj, 1)
    #
    # for i = 1:1:ltr.s.sampling_number
    #     temp_new_node = TPPNode(traj[i, 1:3], traj[i, 4:6], traj[i, 7], traj[i, 8])
    #     push!(ltr.p.nodes_collection,temp_new_node)
    # end
    ltr.p.nodes_collection = deepcopy(ltr.p.traj)
    ltr.s.starting_position = traj[1, 1:3]
    ltr.s.ending_position = traj[end, 1:3]
    ltr.s.BoundPosition = BoundPosition
    ltr.p.costs_collection = traj[:, 8]
    ltr.p.states_collection = [traj[:, 1:3] traj[:, 6] traj[:, 7]]
    Classifier = matread("src/ClassifierInfo.mat");
    LayerInfo = Classifier["LayerInfo"]
    ltr.s.LayerInfo = LayerInfo
    ltr.p.starting_node = start_node
    ltr.p.ending_node = end_node
end
function defineLTRobs!(ltr::LTRSearcher, obstacle_list)
    ltr.s.obstacle_list = obstacle_list
    return nothing
end


function defineLTRterrain!(ltr::LTRSearcher, terrain_kdtree::KDTree, pointcloud::Matrix{Float64})
    ltr.s.terrain_info = pointcloud
    ltr.s.terrain_kdtree = terrain_kdtree
    return nothing
end

function defineLTRKeyParams(ltr::LTRSearcher, iterations, delta_dist, delta_speed, delta_yaw)
    ltr.s.iterations = iterations
    ltr.s.delta_dist = delta_dist
    ltr.s.delta_speed = delta_speed
    ltr.s.delta_yaw = delta_yaw
end
