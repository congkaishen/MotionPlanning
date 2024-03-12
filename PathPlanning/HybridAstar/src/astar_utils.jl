function circleShape(h,k,r)
    θ = LinRange(0, 2*π, 500)
    h.+r*sin.(θ), k.+r*cos.(θ)
end

function regulate_states(hybrid_astar::HybridAstarSearcher, states)
    st_reso = hybrid_astar.s.resolutions
    x_res = st_reso[1]
    y_res = st_reso[2]
    ψ_res = st_reso[3]
    
    x = round(states[1]/x_res)*x_res
    y = round(states[2]/y_res)*y_res
    ψ_ori = modπ(states[3])
    ψ = round(ψ_ori/ψ_res)*ψ_res
    return [x,y,ψ]
end

function RS_connected(hybrid_astar::HybridAstarSearcher, current_node::HybridAstarNode)
    cur_st = current_node.states
    goal_st = hybrid_astar.s.ending_states
    minR = hybrid_astar.s.minR
    block_list = hybrid_astar.s.obstacle_list
    norm_states = changeBasis(cur_st, goal_st, minR)
    opt_cmd, opt_cost, _, _ = allpath(norm_states)
    path = createActPath(cur_st, minR, opt_cmd)
    return block_collision_check(path, block_list), path
end


function block_outside(states, block_info)
    x = states[1]
    y = states[2]
    block_center_x = block_info[1]
    block_center_y = block_info[2]
    block_yaw = block_info[3]
    block_length = block_info[4]
    block_width = block_info[5]
    p =4.0
    return -((((cos(block_yaw) * (x-block_center_x) + sin(block_yaw) * (y-block_center_y))/block_length )^p + ( (-sin(block_yaw) * (x-block_center_x) + cos(block_yaw) * (y-block_center_y))/(block_width) )^p + 0.01)^(1/p) ) + 1 <= 0 
end

# function block_outside(states, block_info)
#     x = states[1]
#     y = states[2]
#     block_center_x = block_info[1]
#     block_center_y = block_info[2]
#     block_yaw = 0.0
#     block_length = block_info[3]
#     block_width = block_info[3]
#     p =2.0
#     return -((((cos(block_yaw) * (x-block_center_x) + sin(block_yaw) * (y-block_center_y))/block_length )^p + ( (-sin(block_yaw) * (x-block_center_x) + cos(block_yaw) * (y-block_center_y))/(block_width) )^p + 0.01)^(1/p) ) + 1 <= 0 
# end

function block_collision_check(path, block_list)
    if size(path, 2) > 15
        x = path[1, 1:15:end]
        y = path[2, 1:15:end]
    else
        x = path[1,1]
        y = path[2,1]
    end
    for i in 1:size(block_list, 1)
        for j in 1:size(x, 1)
            if !block_outside([x[j] y[j]], block_list[i])
                return false
            end
        end
    end
    return true
end

function planHybridAstar!(hybrid_astar::HybridAstarSearcher)
    t1 = time()
    push!(hybrid_astar.p.open_list, hybrid_astar.p.starting_node)
    while !isempty(hybrid_astar.p.open_list)
        hybrid_astar.p.loop_count = hybrid_astar.p.loop_count + 1
        sort!(hybrid_astar.p.open_list)
        current_node = popfirst!(hybrid_astar.p.open_list)

        # if hybrid_astar.s.draw_fig == true && mod(hybrid_astar.p.loop_count, 10)==1
        #     retrievepath(hybrid_astar, current_node) 
        #     # println(size(hybrid_astar.r.actualpath))
        #     h = plotRes(hybrid_astar)
        #     display(h)
        #     sleep(0.001)
        # end

        termi_flag, final_path = RS_connected(hybrid_astar, current_node)

        if termi_flag
            actualpath = final_path
            hybrid_astar_states = current_node.states


            current_node = hybrid_astar.p.nodes_collection[current_node.parent]
            while current_node != nothing
                hybrid_astar_states = [hybrid_astar_states current_node.states]
                if current_node.parent == nothing
                    current_node = nothing
                else
                    current_node = hybrid_astar.p.nodes_collection[current_node.parent]
                end
            end
            
            hybrid_astar.r.hybrid_astar_states = hybrid_astar_states
            hybrid_astar.r.actualpath  = actualpath

            @goto escape_label
        end

        FindNewNode(hybrid_astar, current_node)
    end

    @label escape_label
    t2 = time()
    hybrid_astar.r.planning_time = t2 - t1
    return nothing
end



function InOpen(hybrid_astar::HybridAstarSearcher, current_node::HybridAstarNode)
    for node in hybrid_astar.p.open_list
        if current_node.index == node.index
            return true
        end
    end
    return false
end


function InClose(hybrid_astar::HybridAstarSearcher, current_node::HybridAstarNode)
    for node in hybrid_astar.p.closed_list
        if current_node.index == node.index
            return true
        end
    end
    return false
end



function Encode(hybrid_astar::HybridAstarSearcher, states)
    states_bounds = hybrid_astar.s.stbound
    resolutions = hybrid_astar.s.resolutions

    x_range = states_bounds[1,:]
    y_range = states_bounds[2,:]
    ψ_range = states_bounds[3,:]
    
    x = states[1]
    y = states[2]
    ψ = states[3]

    x = max(min(x, x_range[2]), x_range[1])
    y = max(min(y, y_range[2]), y_range[1])
    ψ = max(min(ψ, ψ_range[2]), ψ_range[1])

    x_res = resolutions[1]
    y_res = resolutions[2]
    ψ_res = resolutions[3]

    x_id = round((x - x_range[1])/x_res) + 1
    y_id = round((y - y_range[1])/y_res) + 1
    ψ_id = round((ψ - ψ_range[1])/ψ_res) + 1

    y_num = round((y_range[2] - y_range[1])/y_res) + 1
    ψ_num = round((ψ_range[2] - ψ_range[1])/ψ_res) + 1
    idx = (x_id - 1)*y_num*ψ_num + (y_id - 1)*ψ_num + ψ_id

    if check_bounds(hybrid_astar, states)
        return Int64(idx)
    else
        return Int64(0)
    end
    
end

function dg_cost(hybrid_astar::HybridAstarSearcher,  path)
    #### add obs collision
    block_list = hybrid_astar.s.obstacle_list
    if !block_collision_check(path, block_list)
        return Inf
    end
    return hybrid_astar.s.expand_time
end

function rs_heuristic(hybrid_astar::HybridAstarSearcher,   cur_st)
    goal_st = hybrid_astar.s.ending_states
    minR = hybrid_astar.s.minR
    # Option 1
    norm_states = changeBasis(cur_st, goal_st, minR)
    _, opt_cost, _, _ = allpath(norm_states)
    cost = opt_cost*minR

    # Option 2
    # cost = sqrt((cur_st[1]- goal_st[1])^2 + (cur_st[2]- goal_st[2])^2 + (cur_st[3]- goal_st[3])^2)
    # cost = sqrt((cur_st[1]- goal_st[1])^2 + (cur_st[2]- goal_st[2])^2)
    return cost
end

function FindNewNode(hybrid_astar::HybridAstarSearcher, current_node::HybridAstarNode)
    cur_idx = Encode(hybrid_astar, current_node.states)
    cur_st = current_node.states
    
    neighbors_st = transform(cur_st, hybrid_astar.s.states_candi)
    neighbors_path = transform(cur_st, hybrid_astar.s.paths_candi)


    for neighbor_idx in 1:hybrid_astar.s.num_neighbors
        
        nb_st = regulate_states(hybrid_astar, neighbors_st[:, neighbor_idx])   # 3 x 1
        nb_path = neighbors_path[:,:,neighbor_idx] # 3 x N [x;y;ψ]

        neighbor_idx = Encode(hybrid_astar, nb_st)
        need_update = false

        if neighbor_idx == 0
            continue
        end

        temp_g =current_node.g + dg_cost(hybrid_astar, nb_path)
        temp_h = rs_heuristic(hybrid_astar, nb_st)
        temp_f = temp_g + temp_h

        if haskey(hybrid_astar.p.nodes_collection, neighbor_idx)
            temp_node = hybrid_astar.p.nodes_collection[ Encode(hybrid_astar, nb_st) ]
            if temp_g<temp_node.g
                temp_node.g = temp_g
                temp_node.h = temp_h
                temp_node.f = temp_f
                temp_node.parent = cur_idx
                need_update = true
            else
                need_update = false
            end
        else
            temp_node = HybridAstarNode(cur_idx, nb_st, neighbor_idx, temp_g, temp_h, temp_f)
            hybrid_astar.p.nodes_collection[neighbor_idx] = temp_node
            need_update = true
        end

        if need_update
            if !InOpen(hybrid_astar, temp_node)
                 push!(hybrid_astar.p.open_list, temp_node)
            end
        end
    end
    push!(hybrid_astar.p.closed_list, current_node)
end


function check_bounds(hybrid_astar::HybridAstarSearcher, states)
    states_bounds = hybrid_astar.s.stbound
    for i in 1:3
        if states[i] < states_bounds[i,1] || states[i] > states_bounds[i,2]
            return false
        end
    end
    return true
end


function transform(init_st, states)
    res_states = zeros(size(states))
    θ = init_st[3]
    x = init_st[1]
    y = init_st[2]
    if size(states, 3) == 1
        if size(states, 2) == 1
            res_states[1] = states[1]*cos(θ) - states[2]*sin(θ) + x
            res_states[2] = states[1]*sin(θ) + states[2]*cos(θ) + y
            res_states[3] = states[3] + θ
        else
            res_states[1,:] = states[1,:]*cos(θ) .- states[2,:]*sin(θ) .+ x
            res_states[2,:] = states[1,:]*sin(θ) .+ states[2,:]*cos(θ) .+ y
            res_states[3,:] = states[3,:] .+ θ
        end
    else
        res_states[1,:,:] = states[1,:,:]*cos(θ) .- states[2,:,:]*sin(θ) .+ x
        res_states[2,:,:] = states[1,:,:]*sin(θ) .+ states[2,:,:]*cos(θ) .+ y
        res_states[3,:,:] = states[3,:,:] .+ θ
    end

    return res_states
end


function neighbor_origin(T, steer_set, gear_set) #init at [0,0,0] x,y,ψ
    Δt = 1e-2
    num_col = Int32(floor(T/Δt))
    num_steer = size(steer_set, 1)
    num_gear = size(gear_set, 1)
    states_candi = zeros(3, num_steer*num_gear)
    paths_candi = zeros(3, num_col, num_steer*num_gear)
    for gear_idx in 1:num_gear
        for steer_idx in 1:num_steer
            states = [0,0,0]
            for i = 1:num_col
                ctrls = [gear_set[gear_idx], steer_set[steer_idx]]
                dstates = simCarModel(states, ctrls)
                states = states + dstates*Δt
                paths_candi[:,i,(gear_idx-1)*num_steer+steer_idx] = states
            end
            states_candi[:,(gear_idx-1)*num_steer+steer_idx] = states
        end
    end
    return states_candi, paths_candi
end