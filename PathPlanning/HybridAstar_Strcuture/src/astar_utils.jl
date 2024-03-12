function circleShape(h,k,r)
    θ = LinRange(0, 2*π, 500)
    h.+r*sin.(θ), k.+r*cos.(θ)
end


function plotRes(hybrid_astar)
	goal_pt = hybrid_astar.s.ending_states
    obs_setting = hybrid_astar.s.obstacle_list
	h = plot(size = [1000, 600])
    if size(hybrid_astar.r.actualpath, 1) > 2
        for (key, node) in hybrid_astar.p.nodes_collection
            act_states = TransferCoordinate(hybrid_astar, node.states)
            h = scatter!(h, [act_states[1]], [act_states[2]], c=:gray, fillalpha = 0.1)
        end
        h = plot!(h, hybrid_astar.r.actualpath[:,1], hybrid_astar.r.actualpath[:,2],aspect_ratio=:equal, lc=:red, legend=false)
    end
    for obs_idx = 1:1:size(obs_setting, 1)
        h = plot!(h, circleShape(obs_setting[obs_idx][1], obs_setting[obs_idx][2], obs_setting[obs_idx][3]), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 1.0)
    end
	h = plot!(h, circleShape(goal_pt[1], goal_pt[2], 0.25), seriestype = [:shape,], ;w = 0.5, c=:green, linecolor = :green, legend = false, fillalpha = 1.0)
    return h

end



function retrievepath(hybrid_astar::HybridAstarSearcher, ori_current_node::HybridAstarNode) 
    hybrid_astarpath = ori_current_node.states
    if ori_current_node.parent == nothing
        hybrid_astar.r.actualpath = TransferCoordinate(hybrid_astar, ori_current_node.states)
        return nothing
    end
    current_node = hybrid_astar.p.nodes_collection[ori_current_node.parent]
    while current_node != nothing
        hybrid_astarpath = [hybrid_astarpath current_node.states]
        if current_node.parent == nothing
            current_node = nothing
        else
            current_node = hybrid_astar.p.nodes_collection[current_node.parent]
        end
    end
    hybrid_astarpath = [reverse!(hybrid_astarpath[1, :]) reverse!(hybrid_astarpath[2, :])]
    hybrid_astar.r.hybrid_astarpath = hybrid_astarpath
    actualpath = deepcopy(hybrid_astarpath)
    actualpath = convert(Matrix{Float64}, actualpath)
    for i = 1:size(actualpath, 1)
        states_val = actualpath[i, :]
        actualpath[i,:] = TransferCoordinate(hybrid_astar, states_val)
    end
    hybrid_astar.r.actualpath  = actualpath
end

function planHybridAstar!(hybrid_astar::HybridAstarSearcher)
    t1 = time()
    push!(hybrid_astar.p.open_list, hybrid_astar.p.starting_node)
    while !isempty(hybrid_astar.p.open_list)
        hybrid_astar.p.loop_count = hybrid_astar.p.loop_count + 1
        sort!(hybrid_astar.p.open_list)
        current_node = popfirst!(hybrid_astar.p.open_list)

        if hybrid_astar.s.draw_fig == true && mod(hybrid_astar.p.loop_count, 10)==1
            retrievepath(hybrid_astar, current_node) 
            # println(size(hybrid_astar.r.actualpath))
            h = plotRes(hybrid_astar)
            display(h)
            sleep(0.001)
        end



        if current_node.states == hybrid_astar.s.ending_states
            hybrid_astarpath = hybrid_astar.s.ending_states
            current_node = hybrid_astar.p.nodes_collection[current_node.parent]
            while current_node != nothing
                hybrid_astarpath = [hybrid_astarpath current_node.states]
                if current_node.parent == nothing
                    current_node = nothing
                else
                    current_node = hybrid_astar.p.nodes_collection[current_node.parent]
                end
            end
            
            hybrid_astarpath = [reverse!(hybrid_astarpath[1, :]) reverse!(hybrid_astarpath[2, :])]
            hybrid_astar.r.hybrid_astarpath = hybrid_astarpath
            actualpath = deepcopy(hybrid_astarpath)
            actualpath = convert(Matrix{Float64}, actualpath)
            for i = 1:size(actualpath, 1)
                states_val = actualpath[i, :]
                actualpath[i,:] = TransferCoordinate(hybrid_astar, states_val)
            end
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
        if current_node.states == node.states
            return true
        end
    end
    return false
end


function InClose(hybrid_astar::HybridAstarSearcher, current_node::HybridAstarNode)
    for node in hybrid_astar.p.closed_list
        if current_node.states == node.states
            return true
        end
    end
    return false
end

function RemoveClose(hybrid_astar::HybridAstarSearcher, current_node::HybridAstarNode)
    idx = 0

    for node in hybrid_astar.p.closed_list
        idx = idx+1
        if current_node.states == node.states
            deleteat!(hybrid_astar.p.closed_list, Int64(idx))
        end
    end
end


function Encode(hybrid_astar::HybridAstarSearcher, states::Vector{Int64})
    x = states[1]
    y = states[2]
    XMAX = hybrid_astar.s.mapbound[1]
    YMAX = hybrid_astar.s.mapbound[2]

    if x<1 || x>XMAX || y<1 || y>YMAX
        return Int64(0)
    else
        hybrid_astar_idx = (y-1)*XMAX + x
        return Int64(floor(hybrid_astar_idx))
    end
end

function FindNewNode(hybrid_astar::HybridAstarSearcher, current_node::HybridAstarNode)
    cur_idx = Encode(hybrid_astar, current_node.states)
    directions = [(-1,0),(-1,-1), (-1,-1), (1,-1), (1,0),(1,1), (0,-1), (1,-1), (0,1)]
    # directions = [(-1,0),(1,0),(0,-1), (0,1)]
    for direction in directions
        new_states = [ Int64(current_node.states[1] + direction[1]), Int64(current_node.states[2] + direction[2])]

        neighbor_idx = Encode(hybrid_astar, new_states)
        need_update = false

        if checkPositionValidity(hybrid_astar, new_states)
            if neighbor_idx == 0
                continue
            end
            temp_g =current_node.g + sqrt((new_states[1] - current_node.states[1])^2*hybrid_astar.s.x_factor^2 + (new_states[2] - current_node.states[2])^2*hybrid_astar.s.y_factor^2 )
            temp_h = sqrt((new_states[1] - hybrid_astar.s.ending_states[1])^2*hybrid_astar.s.x_factor^2 + (new_states[2] - hybrid_astar.s.ending_states[2])^2*hybrid_astar.s.y_factor^2)
            temp_f = temp_g + temp_h

            if haskey(hybrid_astar.p.nodes_collection, neighbor_idx)
                temp_node = deepcopy(hybrid_astar.p.nodes_collection[ Encode(hybrid_astar, new_states) ])
                if temp_g<temp_node.g
                    need_update = true
                else
                    need_update = false
                end
            else
                temp_node = HybridAstarNode(cur_idx, new_states, temp_g, temp_h, temp_f)
                temp_node.g = temp_g
                temp_node.h = temp_h
                temp_node.f = temp_f
                temp_node.parent = cur_idx
                hybrid_astar.p.nodes_collection[neighbor_idx] = temp_node
                need_update = true
            end

            if need_update
                temp_node.g = temp_g
                temp_node.h = temp_h
                temp_node.f = temp_f
                temp_node.parent = cur_idx
                hybrid_astar.p.nodes_collection[neighbor_idx] = temp_node

                if !InOpen(hybrid_astar, temp_node)
                     push!(hybrid_astar.p.open_list, temp_node)
                end

            end




            # if InOpen(hybrid_astar, temp_node)
            #     if !need_update
            #         continue
            #     end
            # elseif InClose(hybrid_astar, temp_node)
            #     if !need_update
            #         continue
            #     end
            #
            #     RemoveClose(hybrid_astar, temp_node)
            #     temp_node.f = temp_f
            #     push!(hybrid_astar.p.open_list, temp_node)
            # else
            #     temp_node.g = temp_g
            #     temp_node.h = temp_h
            #     temp_node.f = temp_f
            #     temp_node.parent = cur_idx
            #     hybrid_astar.p.nodes_collection[neighbor_idx] = temp_node
            # end


        end

    end
    push!(hybrid_astar.p.closed_list, current_node)
end

function TransferCoordinate(hybrid_astar::HybridAstarSearcher, states_val)
    new_states = [ (states_val[1] - 1.0) * hybrid_astar.s.x_factor + hybrid_astar.s.actualbound[1]; (states_val[2] - 1.0) * hybrid_astar.s.y_factor + hybrid_astar.s.actualbound[3]]
    return new_states
end


function checkPositionValidity(hybrid_astar::HybridAstarSearcher, states_val)
    flag = true
    if states_val[1] < 1 || states_val[2] < 1 || states_val[1] > hybrid_astar.s.mapbound[1] || states_val[2] > hybrid_astar.s.mapbound[2]
        flag = false
    end
    obsflag = checkObsSafety(hybrid_astar, states_val)
    return flag && obsflag
end


function checkObsSafety(hybrid_astar, states_val)
    flag = true
    transferred_states_val = TransferCoordinate(hybrid_astar, states_val)
    for i = 1:size(hybrid_astar.s.obstacle_list, 1)
        if (transferred_states_val[1] - hybrid_astar.s.obstacle_list[i][1])^2 + (transferred_states_val[2] - hybrid_astar.s.obstacle_list[i][2])^2 < hybrid_astar.s.obstacle_list[i][3]^2
            flag = false
            break;
        end
    end
    return flag
end
