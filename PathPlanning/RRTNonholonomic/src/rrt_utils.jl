using Plots
using DelimitedFiles
using ProgressBars
using Distributed
using LinearAlgebra
using Shuffle
using Parameters
using NearestNeighbors
using Statistics
using Distances
using MAT
using ProgressMeter
using Distributions
using StaticArrays
include("types.jl")
include("plotting_utils.jl")
include("setup.jl")
function sampleRRT(rrt::RRTSearcher)::RRTNode
    max_x = rrt.s.BoundPosition[2]
    min_x = rrt.s.BoundPosition[1]
    max_y = rrt.s.BoundPosition[4]
    min_y = rrt.s.BoundPosition[3]

    bias_rate = rrt.s.bias_rate
    rand_num = rand(1)[1]

    if rand_num < bias_rate
        xpos = rrt.p.ending_node.loc[1]
        ypos = rrt.p.ending_node.loc[2]
        zpos = 0.0
        ψ = (rand(1)[1]-0.5)*2*pi
    else
        xpos = rand(1).*(max_x-min_x).+min_x
        ypos = rand(1).*(max_y-min_y).+min_y
        zpos = 0.0
        ψ = (rand(1)[1]-0.5)*2*pi
    end
    zpos = 0

    newloc = [xpos;ypos]

    node = RRTNode(newloc, ψ, 0.0)
    return node
end


function euler2Rot(eulerang::Float64)::Matrix{Float64}
    ψ =eulerang
    R = [cos(ψ) -sin(ψ) 0;
    sin(ψ) cos(ψ) 0;
    0 0 1]
    return R
end


function getChildYaw(args...)::Float64
    if length(args) == 2
        startnode = args[1]
        new_node = args[2]
        pos1 = startnode.loc
        pos2 = new_node.loc
        R1 = euler2Rot(startnode.eulerang)
        R2 = euler2Rot(new_node.eulerang)
    elseif length(args) == 1
        new_node = args[1][:, 2]
        startnode = args[1][:, 1]
        pos1 = startnode[1:3]
        pos2 = new_node[1:3]
        R1 = euler2Rot(startnode[4:6])
        R2 = euler2Rot(new_node[4:6])
    end
    h1 = R1[:,1]
    n1 = R1[:,3]
    zpos = -(n1[1]*(pos2[1]-pos1[1])+n1[2]*(pos2[2]-pos1[2]))/n1[3]+pos1[3]
    proj_loc = [pos2[1]; pos2[2]; zpos]
    dis_vec = proj_loc-pos1


    if norm(dis_vec) >= 0.001
        dis_vec = dis_vec/norm(dis_vec)
        Δψ_sign = cross(h1, dis_vec)
        Δψ = acos( round(dot(dis_vec, h1)/(norm(dis_vec)*norm(h1)), digits = 2  ) )*sign(Δψ_sign[3])
    else
        Δψ = 0
    end
    return Δψ
end



function getNormalVec(rrt, pos2::Vector{Float64}, idxs::Vector{Int64})::Vector{Float64}
    p_list =  rrt.s.terrain_info[:,idxs]
    cov_matrix  =cov(p_list, dims=2)
    sconv_matrix = SMatrix{3,3}(cov_matrix)
    eigen_result = eigen(sconv_matrix)
    svecs = eigen_result.vectors
    normalvec = [svecs[1,1]; svecs[2,1]; svecs[3,1]]
    normalvec = normalvec*sign(normalvec[3])
    return normalvec
end





function neural_cost(states)
    X = deepcopy(states)
    distCost = sqrt(X[1]^2 + X[2]^2)
    AngleCost = states[3]^2 / distCost
    cost = 5 * distCost + AngleCost
    return cost
end


function get_max_ang_ratio(dist)
    max_ang = 0.1
    return max_ang/dist
end

function get_max_yawangle(x::Float64, y::Float64)
    p00 =    -0.03911
    p10 =     0.05831
    p01 =      0.1148
    p20 =    -0.04344
    p11 =      0.4135
    p02 =    -0.05552
    p30 =    0.005515
    p21 =    -0.05701
    p12 =      0.1892
    p03 =     -0.1725
    z1 = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 + p21*x^2*y + p12*x*y^2 + p03*y^3;

    p00 =     0.03911
    p10 =    -0.05831
    p01 =      0.1148
    p20 =     0.04344
    p11 =      0.4135
    p02 =     0.05552
    p30 =   -0.005515
    p21 =    -0.05701
    p12 =     -0.1892
    p03 =     -0.1725
    z2 = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 + p21*x^2*y + p12*x*y^2 + p03*y^3;
    return [z1;z2]
end

function DistToDeltaXY(dist, max_angle_ratio)
    max_angle = max_angle_ratio * dist
    Δx = dist * cos(max_angle)
    Δy = dist * sin(max_angle)
    return [Δx, Δy]
end

function FeasibilityClassifier(x1, planner)
    if size(x1, 1) >5
        x1 = x1[1:5]
    end

    dist = sqrt(x1[1]^2 +x1[2]^2)

    θ = atan(x1[2]/x1[1])
    Δψ = x1[3]

    if θ > abs(get_max_ang_ratio(dist)*dist)
        return false
    end

    ψ_bounds = get_max_yawangle(dist, θ)
    if Δψ < ψ_bounds[1] || Δψ > ψ_bounds[2]
        return false
    end


    return true
end

function getChildYaw(args...)::Float64
    if length(args) == 2
        startnode = args[1]
        new_node = args[2]
        pos1 = startnode.loc
        pos2 = new_node.loc
        R1 = euler2Rot(startnode.eulerang)
        R2 = euler2Rot(new_node.eulerang)
    elseif length(args) == 1
        new_node = args[1][:, 2]
        startnode = args[1][:, 1]
        pos1 = startnode[1:3]
        pos2 = new_node[1:3]
        R1 = euler2Rot(startnode[4:6])
        R2 = euler2Rot(new_node[4:6])
    end
    h1 = R1[:,1]
    dis_vec = pos2-pos1
    dis_vec = dis_vec/norm(dis_vec)
    if norm(dis_vec) >= 0.001
        rho = norm(dis_vec)
        dis_vec = dis_vec/norm(dis_vec)
        Δψ_sign = cross(h1, dis_vec)
        Δψ = acos( round(dot(dis_vec, h1)/(norm(dis_vec)*norm(h1)), digits = 2  ) )*sign(Δψ_sign[3])
    else
        rho = 0
        Δψ = 0
    end
    return Δψ
end

function ProjectionStates(node1, node2)

    pos1 = node1.loc
    pos2 = node2.loc
    eulerang1 = node1.eulerang
    eulerang2 = node2.eulerang
    R1 = euler2Rot(eulerang1)


    h1 = R1[1:2,1]

    proj_loc = [pos2[1]; pos2[2]]
    dis_vec = proj_loc-pos1

    if norm(dis_vec) >= 0.001
        rho = norm(dis_vec)
        dis_vec = dis_vec/norm(dis_vec)
        Δψ_sign = h1[1] * dis_vec[2] - h1[2] * dis_vec[1]
        Δψ = acos( round(dot(dis_vec, h1)/(norm(dis_vec)*norm(h1)), digits = 2  ) )*sign(Δψ_sign)
    else
        rho = 0
        Δψ = 0
    end

    Δx = round(rho*cos(Δψ), digits = 2)
    Δy = round(rho*sin(Δψ), digits = 2)
    Δψ_head = round(eulerang2- eulerang1, digits = 3)

    states = [Δx;Δy;Δψ_head]
    return states
end

function InitialCheck(states)
    Δx = states[1]
    Δy = states[2]
    Δψ_head = states[3]

    if Δx<1.0 || Δx>3.0
        return false
    end

    if Δy<-1.0 || Δy>1.0
        return false
    end

    if Δψ_head<-0.35 || Δψ_head>0.35
        return false
    end

    return true
end


function FromProj2Real(node1, node2, proj_states,  rrt::RRTSearcher)
    pos1 = node1.loc
    R1 = euler2Rot(node1.eulerang)
    h1 = R1[:,1]
    n1 = R1[:,3]
    l1 = cross(n1, h1)
    xy_loc = pos1[1:2]
    xy_loc = xy_loc + h1[1:2]*proj_states[1]
    xy_loc = xy_loc + l1[1:2]*proj_states[2]

    xpos = xy_loc[1]
    ypos = xy_loc[2]
    ψ = node1.eulerang + proj_states[3]
    newloc = [xpos;ypos]
    node2.loc = newloc
    node2.eulerang = ψ
end




function decide_proj(dist::Float64, ang_ratio::Float64, startnode::RRTNode, new_node::RRTNode)::Vector{Float64}

    ΔXY = DistToDeltaXY(dist, ang_ratio)
    acceptable_ψ_bounds = get_max_yawangle(dist, ang_ratio*dist)

    acceptable_ψ = rand(1)[1]*(acceptable_ψ_bounds[2]-acceptable_ψ_bounds[1])+acceptable_ψ_bounds[1]
    proj_states = [ΔXY[1]; ΔXY[2]; acceptable_ψ]
    return proj_states
end

function steer(rrt::RRTSearcher, startnode::RRTNode, targetnode::RRTNode)::RRTNode
    RANDOM = true
    new_node = deepcopy(targetnode)
    min_dist = rrt.s.grow_dist[1]
    maxdist = rrt.s.grow_dist[2]
    proj_states = ProjectionStates(startnode, new_node)
    dist = sqrt(proj_states[1] ^2 + proj_states[2] ^2)
    # TODO Dist within range
    if dist <= maxdist && dist >= min_dist
        max_ang_ratio = get_max_ang_ratio(dist)

        if abs(proj_states[1])<= 0.01
            proj_states[1] = 0.01
        end
        Δψ = atan(proj_states[2]/proj_states[1])
        if abs(Δψ)/dist > max_ang_ratio
            if RANDOM
                ang_ratio = (2*rand(1)[1]-1)*max_ang_ratio
            else
                ang_ratio = max_ang_ratio*sign(Δψ)
            end
            proj_states = decide_proj(dist, ang_ratio, startnode, new_node)
            FromProj2Real(startnode, new_node, proj_states,  rrt)

        else
            ang_ratio = Δψ/dist
            proj_states = decide_proj(dist, ang_ratio, startnode, new_node)
            FromProj2Real(startnode, new_node, proj_states,  rrt)
        end
        proj_states = ProjectionStates(startnode, new_node)
        new_node.cost = startnode.cost +  neural_cost(proj_states)
        return new_node
    else
        # if RANDOM
        #     adjustdist = rand(1)[1]*(maxdist-mindist)+mindist
        # else
        #     adjustdist = maxdist * (dist > maxdist) + min_dist * (dist < min_dist)
        # end

        adjustdist = maxdist
        # adjustdist = mindist

        vec2 = [proj_states[1], proj_states[2]]
        if dist > 0.001
            tuned_vec2 = vec2.* (adjustdist/dist)
            tuned_proj_states = [tuned_vec2[1], tuned_vec2[2], proj_states[3]]
            FromProj2Real(startnode, new_node, tuned_proj_states,  rrt)
        end

        proj_states = ProjectionStates(startnode, new_node)
        max_ang_ratio = get_max_ang_ratio(adjustdist)

        if abs(proj_states[1])<= 0.01
            proj_states[1] = 0.01
        end
        Δψ = atan(proj_states[2]/proj_states[1])
        if abs(Δψ)/adjustdist > max_ang_ratio
            if RANDOM
                ang_ratio = (2*rand(1)[1]-1)*max_ang_ratio
            else
                ang_ratio = max_ang_ratio*sign(Δψ)
            end
            proj_states = decide_proj(adjustdist, ang_ratio, startnode, new_node)
            FromProj2Real(startnode, new_node, proj_states,  rrt)
        else
            ang_ratio = Δψ/adjustdist
            proj_states = decide_proj(adjustdist, ang_ratio, startnode, new_node)
            FromProj2Real(startnode, new_node, proj_states,  rrt)
        end
        proj_states = ProjectionStates(startnode, new_node)
        new_node.cost = startnode.cost +  neural_cost(proj_states)
        return new_node
    end
end



############################
function nearDisk(rrt::RRTSearcher)::Float64
    gamma = 6*(rrt.s.BoundPosition[2]-rrt.s.BoundPosition[1])*(rrt.s.BoundPosition[4]-rrt.s.BoundPosition[3])*5
    return minimum([gamma*(log(rrt.p.sample_idx)/rrt.p.sample_idx),  24])
end


function chooseParents(rrt::RRTSearcher, new_node::RRTNode, nearIdxs::Vector{Int64})::Int64
    mincost = Inf
    minIdx = -1
    for nearIdx in nearIdxs
        proj_states = ProjectionStates(rrt.p.nodes_collection[nearIdx], new_node)
        tempcost = rrt.p.costs_collection[nearIdx]+ neural_cost(proj_states)
        if tempcost <= mincost && !collision(rrt.s.obstacle_list, rrt.p.nodes_collection[nearIdx], new_node, rrt)
            minIdx = nearIdx
            mincost = tempcost
        end
    end
    new_node.cost = mincost
    return minIdx
end


function rewire(rrt::RRTSearcher, new_node::RRTNode, nearIdxs::Vector{Int64}, root_idx::Int64)
    updateIdx_list = Vector{Int64}()
    costchange_list = Vector{Float64}()
    for nearIdx in nearIdxs
        if nearIdx !== root_idx
            proj_states = ProjectionStates(rrt.p.nodes_collection[nearIdx], new_node)
            tempcost = rrt.p.costs_collection[rrt.p.sample_idx]+ neural_cost(proj_states)
            costchange = tempcost - rrt.p.costs_collection[nearIdx]
            if costchange < 0 && !collision(rrt.s.obstacle_list, new_node, rrt.p.nodes_collection[nearIdx], rrt)
                # costchange = tempcost
                parent_idx = rrt.p.parents_collection[nearIdx]
                delete!(rrt.p.children_collection[parent_idx], nearIdx)
                push!(rrt.p.children_collection[rrt.p.sample_idx], nearIdx)
                rrt.p.parents_collection[nearIdx] = rrt.p.sample_idx
                updateIdx_list = [updateIdx_list; nearIdx]
                costchange_list = [costchange_list; costchange]
            end
        end
    end
    if size(updateIdx_list,1)>0
        costPropogation(rrt, updateIdx_list, costchange_list)
    end
end

function costPropogation(rrt::RRTSearcher, updateIdx_list::Vector{Int64} ,costchange_list::Vector{Float64})
    list_num = size(updateIdx_list, 1)
    for list_idx in 1:list_num
        costchange = costchange_list[list_idx]
        parent_idx = updateIdx_list[list_idx]
        updateIdx = findAllUpdatedChildren(rrt.p.children_collection, parent_idx)
        updateIdx = [updateIdx;parent_idx]
        updateNum = size(updateIdx,1)
        if updateNum>0
            for i in 1:updateNum
                rrt.p.costs_collection[updateIdx[i]] = rrt.p.costs_collection[updateIdx[i]]+costchange
                rrt.p.nodes_collection[updateIdx[i]].cost = rrt.p.nodes_collection[updateIdx[i]].cost+costchange
            end
        end
    end
end

function findAllUpdatedChildren(children_collection::Dict{Int64, Set{Int64}}, parent_idx)::Vector{Int64}
    childrenIdxs = children_collection[parent_idx]
    desired_idxs = Vector{Int64}()
    if !isempty(childrenIdxs)
        for child_idx in childrenIdxs
            desired_idxs =  [desired_idxs;child_idx;findAllUpdatedChildren(children_collection, child_idx)]
        end
        return desired_idxs
    else
        return Vector{Int64}()
    end
end



##############################

function CheckWithinBoundary(loc::Vector{Float64}, BoundPosition)::Bool
    flag = false
    if loc[1]<BoundPosition[1]||loc[1]>BoundPosition[2]||loc[2]<BoundPosition[3]||loc[2]>BoundPosition[4]
        flag = true
    end
    return flag
end

function collision( obstacle_list::Vector{Vector{Float64}}, node1, node2, rrt::RRTSearcher)::Bool

    loc_info1 = node1.loc
    loc_info2 = node2.loc
    ψ2 = node2.eulerang
    ψ1 = node1.eulerang

    if CheckWithinBoundary(loc_info2, rrt.s.BoundPosition)
        return true
    end

    proj_states = ProjectionStates(node1, node2)
    if InitialCheck(proj_states) && FeasibilityClassifier(proj_states,rrt)
        kinematic_feasibility = true
    else
        kinematic_feasibility = false
    end
    if !kinematic_feasibility
        return true
    end



    if size(obstacle_list,1) == 0
        return false
    else
        intervals = 2
        start_pt = loc_info1[1:2]
        end_pt = loc_info2[1:2]
        x_list = range(start_pt[1], end_pt[1], intervals)
        y_list = range(start_pt[2], end_pt[2], intervals)
        for obs_idx in 1:size(obstacle_list,1)
            for j in 1:intervals
                if sum(([x_list[j];y_list[j]]-obstacle_list[obs_idx][1:2]).^2)<=obstacle_list[obs_idx][3]^2
                    return true
                end
            end
        end
        return false
    end
end


function CalculateRRTCost(rrt::RRTSearcher)
    final_idxs = reverse(rrt.r.resultIdxs)
    cost = 0;
    for i = 1:1:size(final_idxs, 1) - 1
        start_node = rrt.p.nodes_collection[final_idxs[i]]
        next_node = rrt.p.nodes_collection[final_idxs[i + 1]]
        proj_states = ProjectionStates(start_node, next_node)
        cost = cost + neural_cost(proj_states)
    end
    rrt.r.cost = cost
end

function registerNode(rrt::RRTSearcher, new_node::RRTNode, parent_idx::Int64)
    rrt.p.sample_idx = rrt.p.sample_idx + 1
    rrt.p.buffer_idx = rrt.p.buffer_idx + 1
    rrt.p.nodes_collection[rrt.p.sample_idx] = deepcopy(new_node)
    rrt.p.states_collection[:,rrt.p.sample_idx] = [new_node.loc; new_node.eulerang]
    rrt.p.parents_collection[rrt.p.sample_idx] = parent_idx
    rrt.p.children_collection[rrt.p.sample_idx] = Set{Int}()
    rrt.p.costs_collection[rrt.p.sample_idx] = new_node.cost
    push!(rrt.p.children_collection[parent_idx], rrt.p.sample_idx)
end





function planRRT!(rrt::RRTSearcher)
    t1 = time()
    sampleNum = rrt.s.sampling_number
    p = Progress(sampleNum-1, 0.5, "RRT Progress...", 60)

    while rrt.p.sample_idx <= sampleNum-1
        # println(rrt.p.sample_idx)
        sample_node = sampleRRT(rrt)
        idxs, min_dist = knn(rrt.p.balltree, [sample_node.loc;sample_node.eulerang], 1, true)
        min_idx = idxs[1]
        closest_node = rrt.p.nodes_collection[min_idx]
        new_node = steer(rrt, closest_node, sample_node)

        if !collision(rrt.s.obstacle_list, closest_node, new_node, rrt)
            next!(p)
            registerNode(rrt, new_node, min_idx)

            if rrt.s.rrt_star
                nearIdxs = inrange(rrt.p.balltree, [new_node.loc;new_node.eulerang], nearDisk(rrt), true)
                if size(nearIdxs, 1) >= 30
                    nearIdxs,~ = knn(rrt.p.balltree, [new_node.loc;new_node.eulerang], 30, true)
                end

                if size(nearIdxs,1) != 0
                    parent_idx = min_idx
                    # parent_idx =  chooseParents(rrt, new_node, nearIdxs)
                    rewire(rrt, new_node, nearIdxs, parent_idx)
                end
            end

            if rrt.p.buffer_idx == rrt.s.buffer_size
                rrt.p.balltree = BallTree(rrt.p.states_collection[:, 1:rrt.p.sample_idx])
                rrt.p.buffer_idx = 0
            end
        end
        t3 = time()
        if t3 - t1 > 100
            break
        end

    end
    rrt.p.kdtree = KDTree(rrt.p.states_collection[1:2, 1:rrt.p.sample_idx])
    ed_idx = findBestIdx(rrt)
    t2 = time()

    if ed_idx > 0
        rrt.r.status = :Solved
        rrt.r.Path = getBestPath(rrt, ed_idx)
        rrt.r.resultIdxs = getBestIdxs(rrt, ed_idx)
    else
        rrt.r.status = :NotSolved
    end
    rrt.r.tSolve = t2 - t1

    return nothing;
end

function findBestIdx(rrt::RRTSearcher)
    idxs = inrange(rrt.p.kdtree, rrt.p.ending_node.loc, rrt.s.goal_tolerance, true)
    if size(idxs, 1) > 0
        cost_idxs = argmin(rrt.p.costs_collection[idxs])
        return idxs[cost_idxs]
    else
        return 0
    end
end

function getBestPath(rrt::RRTSearcher, idx::Int)::Matrix{Float64}
    path = rrt.p.nodes_collection[idx].loc
    while idx != 1
        idx = rrt.p.parents_collection[idx]
        path = [path rrt.p.nodes_collection[idx].loc]
    end
    return path
end

function getBestIdxs(rrt::RRTSearcher, idx::Int)::Vector{Int64}
    idxs = Vector{Int64}[]
    idxs = [idxs; idx]
    while idx != 1
        idx = rrt.p.parents_collection[idx]
        idxs = [idxs; idx]
    end
    return idxs
end

function getTree(rrt::RRTSearcher):: Matrix{Float64}
    first = true
    tree = nothing
    for idx in 1:rrt.p.sample_idx
        if (rrt.p.parents_collection[idx]!==-1)
            p_idx = rrt.p.parents_collection[idx]
            if first
                tree = [rrt.p.states_collection[1:3, idx]  rrt.p.states_collection[1:3, p_idx]]
                first = false
            else
                tree = [tree [rrt.p.states_collection[1:3, idx]  rrt.p.states_collection[1:3, p_idx]]]
            end
        end
    end
    return tree
end


function retrieveTree(rrt::RRTSearcher):: Matrix{Float64}
    first = true
    tree = nothing
    for idx in 1:rrt.p.sample_idx
        if (rrt.p.parents_collection[idx]!=-1)
            p_idx = rrt.p.parents_collection[idx]
            if first
                tree = [rrt.p.states_collection[:, idx]  rrt.p.states_collection[:, p_idx]]
                first = false
            else
                tree = [tree [rrt.p.states_collection[:, idx]  rrt.p.states_collection[:, p_idx]]]
            end
        end
    end
    return tree
end
