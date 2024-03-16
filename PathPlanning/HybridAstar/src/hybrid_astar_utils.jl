function plotRes(hybrid_astar::HybridAstarSearcher)
    title_string = "Iterations: $(hybrid_astar.p.loop_count), Expansions: $(length(hybrid_astar.p.nodes_collection)), Open List: $(size(hybrid_astar.p.open_list, 1))"
    h = plot(size = [600, 600], title = title_string, xlim=(hybrid_astar.s.stbound[1, 1]-2, hybrid_astar.s.stbound[1, 2]+2), ylim=(hybrid_astar.s.stbound[2, 1]-2, hybrid_astar.s.stbound[2, 2]+2))
    h = PlotWall(h, Block2Pts(hybrid_astar.s.obstacle_list))

    if length(hybrid_astar.p.nodes_collection) > 1
        for (key, node) in hybrid_astar.p.nodes_collection
            cur_st = node.states
            arrowsize = 0.05
            quiver!(h, [cur_st[1]], [cur_st[2]], quiver = ([arrowsize*cos(cur_st[3])], [arrowsize*sin(cur_st[3])]),color =:gray, legend = false)
        end
    end

    if size(hybrid_astar.r.actualpath, 1) >= 1
        h = plot!(h, hybrid_astar.r.actualpath[1,:], hybrid_astar.r.actualpath[2,:], color =:green, linewidth = 5)
    else
        if size(hybrid_astar.r.RSpath_final, 1) >= 1
            h = plot!(h, hybrid_astar.r.RSpath_final[1,:], hybrid_astar.r.RSpath_final[2,:], color =:green)

            for i in 1:size(hybrid_astar.r.hybrid_astar_states, 2)
                cur_st = hybrid_astar.r.hybrid_astar_states[:, i]
                arrowsize = 0.1
                quiver!(h, [cur_st[1]], [cur_st[2]], quiver = ([arrowsize*cos(cur_st[3])], [arrowsize*sin(cur_st[3])]),color =:green, legend = false)
                display(h)
                sleep(0.1)
            end
        end
    end

    cur_st = hybrid_astar.s.starting_states
    arrowsize = 0.1
    quiver!(h, [cur_st[1]], [cur_st[2]], quiver = ([arrowsize*cos(cur_st[3])], [arrowsize*sin(cur_st[3])]),color =:red, legend = false, linewidth = 5)

    cur_st = hybrid_astar.s.ending_states
    arrowsize = 0.1
    quiver!(h, [cur_st[1]], [cur_st[2]], quiver = ([arrowsize*cos(cur_st[3])], [arrowsize*sin(cur_st[3])]),color =:green, legend = false, linewidth = 5)

    xlabel!("X [m]")
    ylabel!("Y [m]")
    return h
end

function getVehAnim(hybrid_astar::HybridAstarSearcher)
    anim = Plots.Animation()
    title_string = "Parking Replay"
    h = plot(size = [600, 600], title = title_string, xlim=(hybrid_astar.s.stbound[1, 1]-2, hybrid_astar.s.stbound[1, 2]+2), ylim=(hybrid_astar.s.stbound[2, 1]-2, hybrid_astar.s.stbound[2, 2]+2))
    h = PlotWall(h, Block2Pts(hybrid_astar.s.obstacle_list))
    if size(hybrid_astar.r.actualpath, 1) >= 1
        x_interp = hybrid_astar.r.x_interp
        y_interp = hybrid_astar.r.y_interp
        ψ_interp = hybrid_astar.r.ψ_interp
        steps = 100
        traver_s_list = LinRange(0, hybrid_astar.r.tol_length, steps)
        for i in 1:1:size(traver_s_list, 1)-1
            traver_s = traver_s_list[i]
            traver_s_next = traver_s_list[i+1] 
            travel_list = LinRange(traver_s, traver_s_next, 2)
            h = plot!(h, x_interp(travel_list), y_interp(travel_list), color =:green, linewidth = 5)
            veh_block = [x_interp(travel_list[end]), y_interp(travel_list[end]), ψ_interp(travel_list[end]), hybrid_astar.s.vehicle_size[1]/2,  hybrid_astar.s.vehicle_size[2]/2]
            veh_pts = GetRectanglePts(veh_block)
            h = PlotVehicle(h, veh_pts)
            display(h)
            sleep(0.1)
            Plots.frame(anim)
        end        
        gif(anim, "./gifholder/VehAnim.gif", fps = Int32(ceil(1/(hybrid_astar.r.tol_length/steps/1))))
    else
        println("No solution!!")
        return nothing
    end
end

function retrievePath(hybrid_astar::HybridAstarSearcher)
    RSpath_final = hybrid_astar.r.RSpath_final
    states_ori = hybrid_astar.r.hybrid_astar_states
    states = transpose([reverse!(states_ori[1, :]) reverse!(states_ori[2, :]) reverse!(states_ori[3, :])])
    actualpath = Matrix{Float64}[]
    actualpath = hybrid_astar.s.starting_states

    for st_idx in 1:1:size(states,2)-1
        cur_st = states[:,st_idx]
        goal_st = states[:,st_idx+1]
        minR = hybrid_astar.s.minR
        norm_states = changeBasis(cur_st, goal_st, minR)
        opt_cmd, opt_cost, _, _ = allpath(norm_states)
        actualpath = [actualpath createActPath(cur_st, minR, opt_cmd)]
    end
    actualpath = [actualpath RSpath_final]
    hybrid_astar.r.actualpath = actualpath


    actualpts = hybrid_astar.r.actualpath[1:2, :]
    ds = sqrt.(sum((actualpts[:,2:end] - actualpts[:,1:end-1]).^2, dims = 1))
    path_length = zeros(size(actualpts, 2))
    for i in 2:1:size(actualpts, 2)
        path_length[i] = path_length[i-1] + ds[i-1]
    end
    x_interp = linear_interpolation(path_length, hybrid_astar.r.actualpath[1,:])
    y_interp = linear_interpolation(path_length, hybrid_astar.r.actualpath[2,:])
    ψ_interp = linear_interpolation(path_length, hybrid_astar.r.actualpath[3,:])
    hybrid_astar.r.x_interp = x_interp
    hybrid_astar.r.y_interp = y_interp
    hybrid_astar.r.ψ_interp = ψ_interp
    hybrid_astar.r.tol_length = path_length[end]
end

function GetRectanglePts(block)
    # block is 5 element vector: x,y,ψ,l,w
    ox = block[1]
    oy = block[2]
    ψ  = block[3]
    length = block[4]
    width  = block[5]
    pts = [[-length,width] [-length,-width] [length, -width] [length, width] [-length,width]]
    R = [cos(ψ) -sin(ψ); sin(ψ) cos(ψ)]
    return R*pts .+ [ox; oy]
end

function Block2Pts(block_info)
    pts = zeros(2,5,size(block_info, 1))
    for block_idx = 1:1:size(block_info, 1)
        pts[:, :, block_idx] = GetRectanglePts(block_info[block_idx])
    end
    return pts
end

function PlotWall(h, pts)
    for i in 1:size(pts,3)
        h = plot!(h, pts[1,:,i], pts[2,:,i], seriestype = [:shape,], color =:black, legend = false, aspect_ratio =:equal)
    end
    return h
end

function PlotVehicle(h, pts)
    h = plot!(h, pts[1,:], pts[2,:], linestyle=:dash, color =:green,  legend = false)
    return h
end

function SeparatingAxisTheorem(pts_base, pts_other)
    # false means collision
    for bg_idx = 1:1:(size(pts_base, 2) - 1)

        bg_pt = pts_base[:, bg_idx]
        ed_pt = pts_base[:, bg_idx+1]

        base_vec = ed_pt - bg_pt

        normal_vec = [-base_vec[2]; base_vec[1]]
        
        base_dps = transpose(pts_base .- bg_pt)*normal_vec
        other_dps = transpose(pts_other .- bg_pt)*normal_vec

        min_base = minimum(base_dps)
        max_base = maximum(base_dps)

        min_other = minimum(other_dps)
        max_other = maximum(other_dps)
        
        if ((max_other <= min_base) || (max_base <= min_other))
            return true
        end
    end
    return false
end

function ConvexCollision(pts1, pts2)
    if SeparatingAxisTheorem(pts1, pts2)
        if SeparatingAxisTheorem(pts2, pts1)
            return true
        else
            return false
        end
    else
        return false
    end
end

function block_collision_check(hybrid_astar::HybridAstarSearcher, path, block_list)
    sparcity_num = 5
    if size(path, 2) > sparcity_num
        x = path[1, 1:sparcity_num:end]
        y = path[2, 1:sparcity_num:end]
        ψ = path[3, 1:sparcity_num:end]
    else
        x = path[1,1]
        y = path[2,1]
        ψ = path[3,1]
    end

    veh_his = [[x[i], y[i], modπ(ψ[i]), hybrid_astar.s.vehicle_size[1]/2, hybrid_astar.s.vehicle_size[2]/2] for i in 1:size(x,1)]
    wall_pts = Block2Pts(block_list) # 2x5xNobs
    veh_pts = Block2Pts(veh_his)
    for i in 1:size(wall_pts, 3)
        for j in 1:size(veh_pts, 3)
            if !ConvexCollision(wall_pts[:,:,i], veh_pts[:,:,j])
                return false
            end
        end
    end
    return true
end

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
    return block_collision_check( hybrid_astar, path, block_list), path
end

function planHybridAstar!(hybrid_astar::HybridAstarSearcher)
    t1 = time()
    push!(hybrid_astar.p.open_list, hybrid_astar.p.starting_node)

    if hybrid_astar.s.make_gif anim = Plots.Animation() end
    while !isempty(hybrid_astar.p.open_list)
        hybrid_astar.p.loop_count = hybrid_astar.p.loop_count + 1

        sort!(hybrid_astar.p.open_list)
        current_node = popfirst!(hybrid_astar.p.open_list)
        termi_flag, final_path = RS_connected(hybrid_astar, current_node)

        if (hybrid_astar.s.draw_fig || hybrid_astar.s.make_gif) && mod(hybrid_astar.p.loop_count, 10)==1
            h = plotRes(hybrid_astar)
            if hybrid_astar.s.draw_fig
                display(h)
                sleep(0.001)
            end

            if hybrid_astar.s.make_gif
                Plots.frame(anim)
            end
        end

        if termi_flag
            RSpath_final = final_path
            println("end code")
            hybrid_astar_states = current_node.states
            while (current_node.parent != nothing)&&(current_node.index != hybrid_astar.s.starting_index)
                current_node = hybrid_astar.p.nodes_collection[current_node.parent]
                hybrid_astar_states = [hybrid_astar_states current_node.states]
            end
            hybrid_astar.r.hybrid_astar_states = hybrid_astar_states
            hybrid_astar.r.RSpath_final  = RSpath_final
            retrievePath(hybrid_astar)
            @goto escape_label
        end

        FindNewNode(hybrid_astar, current_node)
    end

    @label escape_label

    if (hybrid_astar.s.draw_fig || hybrid_astar.s.make_gif)
        h = plotRes(hybrid_astar)
        if hybrid_astar.s.draw_fig
            display(h)
            sleep(0.001)
        end
        if hybrid_astar.s.make_gif
            Plots.frame(anim)
        end
    end
    if hybrid_astar.s.make_gif 
        gif(anim, "./gifholder/HybridAstar.gif", fps = 2) 
        getVehAnim(hybrid_astar)
    end

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
    ψ = modπ(states[3]) 

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
    if !block_collision_check(hybrid_astar, path, block_list)
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
    cur_idx = current_node.index
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


        travelcost = dg_cost(hybrid_astar, nb_path)

        if travelcost ==Inf
            continue
        end

        temp_g =current_node.g + travelcost
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
    for i in 1:2
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