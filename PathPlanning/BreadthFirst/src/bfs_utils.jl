function planBFS!(bfs::BFSSearcher)
    t1 = time()
    push!(bfs.p.open_list, bfs.p.starting_node)
    while !isempty(bfs.p.open_list)
        bfs.p.loop_count = bfs.p.loop_count + 1
        # println(bfs.p.loop_count)
        current_node = popfirst!(bfs.p.open_list)


        if  mod(bfs.p.loop_count, 30)==1
            retrievepath(bfs, current_node) 
            # println(size(bfs.r.actualpath))
            h = plotRes(bfs)
            display(h)
            sleep(0.001)
        end


        if current_node.position == bfs.s.ending_pos
            bfspath = bfs.s.ending_pos
            current_node = bfs.p.nodes_collection[current_node.parent]
            while current_node != nothing
                bfspath = [bfspath current_node.position]
                if current_node.parent == nothing
                    current_node = nothing
                else
                    current_node = bfs.p.nodes_collection[current_node.parent]
                end
            end
            
            bfspath = [reverse!(bfspath[1, :]) reverse!(bfspath[2, :])]
            bfs.r.bfspath = bfspath
            actualpath = deepcopy(bfspath)
            actualpath = convert(Matrix{Float64}, actualpath)
            for i = 1:size(actualpath, 1)
                position_val = actualpath[i, :]
                actualpath[i,:] = TransferCoordinate(bfs, position_val)
            end
            bfs.r.actualpath  = actualpath
            h = plot!(bfs.r.actualpath[:,1], bfs.r.actualpath[:,2], lc = :green)
            display(h)
            @goto escape_label
        end

        FindNewNode(bfs, current_node)
    end


    @label escape_label
    t2 = time()
    bfs.r.planning_time = t2 - t1
    return nothing
end


function retrievepath(bfs::BFSSearcher, ori_current_node::BFSNode) 
    bfspath = ori_current_node.position
    if ori_current_node.parent == nothing
        bfs.r.actualpath = TransferCoordinate(bfs, ori_current_node.position)
        return nothing
    end

    current_node = bfs.p.nodes_collection[ori_current_node.parent]

    while current_node != nothing
        bfspath = [bfspath current_node.position]
        if current_node.parent == nothing
            current_node = nothing
        else
            current_node = bfs.p.nodes_collection[current_node.parent]
        end
    end
    
    bfspath = [reverse!(bfspath[1, :]) reverse!(bfspath[2, :])]
    bfs.r.bfspath = bfspath
    actualpath = deepcopy(bfspath)
    actualpath = convert(Matrix{Float64}, actualpath)
    for i = 1:size(actualpath, 1)
        position_val = actualpath[i, :]
        actualpath[i,:] = TransferCoordinate(bfs, position_val)
    end
    bfs.r.actualpath  = actualpath
end


function InOpen(bfs::BFSSearcher, current_node::BFSNode)
    for node in bfs.p.open_list
        if current_node.position == node.position
            return true
        end
    end
    return false
end


function InClose(bfs::BFSSearcher, current_node::BFSNode)
    for node in bfs.p.closed_list
        if current_node.position == node.position
            return true
        end
    end
    return false
end

function RemoveClose(bfs::BFSSearcher, current_node::BFSNode)
    idx = 0

    for node in bfs.p.closed_list
        idx = idx+1
        if current_node.position == node.position
            deleteat!(bfs.p.closed_list, Int64(idx))
        end
    end
end


function Encode(bfs::BFSSearcher, pos::Vector{Int64})
    x = pos[1]
    y = pos[2]
    XMAX = bfs.s.mapbound[1]
    YMAX = bfs.s.mapbound[2]

    if x<1 || x>XMAX || y<1 || y>YMAX
        return Int64(0)
    else
        bfs_idx = (y-1)*XMAX + x
        return Int64(floor(bfs_idx))
    end
end

function FindNewNode(bfs::BFSSearcher, current_node::BFSNode)
    cur_idx = Encode(bfs, current_node.position)
    # directions = [(-1,0),(-1,-1), (-1,-1), (1,-1), (1,0),(1,1), (0,-1), (1,-1), (0,1)]
    directions = [(-1,0),(1,0),(0,-1), (0,1)]
    for direction in directions
        new_position = [ Int64(current_node.position[1] + direction[1]), Int64(current_node.position[2] + direction[2])]

        neighbor_idx = Encode(bfs, new_position)
        need_update = false

        if checkPositionValidity(bfs, new_position)
            if neighbor_idx == 0
                continue
            end

            if !haskey(bfs.p.nodes_collection, neighbor_idx)
                temp_node = BFSNode(cur_idx, new_position)
                bfs.p.nodes_collection[neighbor_idx] = temp_node
                push!(bfs.p.open_list, temp_node)
            end




            # if InOpen(bfs, temp_node)
            #     if !need_update
            #         continue
            #     end
            # elseif InClose(bfs, temp_node)
            #     if !need_update
            #         continue
            #     end
            #
            #     RemoveClose(bfs, temp_node)
            #     temp_node.f = temp_f
            #     push!(bfs.p.open_list, temp_node)
            # else
            #     temp_node.g = temp_g
            #     temp_node.h = temp_h
            #     temp_node.f = temp_f
            #     temp_node.parent = cur_idx
            #     bfs.p.nodes_collection[neighbor_idx] = temp_node
            # end


        end

    end
    push!(bfs.p.closed_list, current_node)
end

function TransferCoordinate(bfs::BFSSearcher, position_val)
    new_position = [ (position_val[1] - 1.0) * bfs.s.x_factor + bfs.s.actualbound[1]; (position_val[2] - 1.0) * bfs.s.y_factor + bfs.s.actualbound[3]]
    return new_position
end


function checkPositionValidity(bfs::BFSSearcher, position_val)
    flag = true
    if position_val[1] < 1 || position_val[2] < 1 || position_val[1] > bfs.s.mapbound[1] || position_val[2] > bfs.s.mapbound[2]
        flag = false
    end
    oBFSlag = checkObsSafety(bfs, position_val)
    return flag && oBFSlag
end


function checkObsSafety(bfs, position_val)
    flag = true
    transferred_position_val = TransferCoordinate(bfs, position_val)
    for i = 1:size(bfs.s.obstacle_list, 1)
        if (transferred_position_val[1] - bfs.s.obstacle_list[i][1])^2 + (transferred_position_val[2] - bfs.s.obstacle_list[i][2])^2 < bfs.s.obstacle_list[i][3]^2
            flag = false
            break;
        end
    end
    return flag
end

function circleShape(h,k,r)
    θ = LinRange(0, 2*π, 500)
    h.+r*sin.(θ), k.+r*cos.(θ)
end


function plotRes(bfs)
	goal_pt = bfs.s.ending_real
    obs_setting = bfs.s.obstacle_list

	h = plot(size = [1000, 600])
	h = plot!(h, circleShape(goal_pt[1], goal_pt[2], 2), seriestype = [:shape,], ;w = 0.5, aspect_ratio=:equal, c=:green, linecolor = :green, legend = false, fillalpha = 1.0)

    if size(bfs.r.actualpath, 1) > 2
        for (key, node) in bfs.p.nodes_collection
            act_pos = TransferCoordinate(bfs, node.position)
            h = scatter!(h, [act_pos[1]], [act_pos[2]], c=:gray, fillalpha = 0.2, markersize=1)
            # h = plot!(h, circleShape(act_pos[1], act_pos[2], 0.25), seriestype = [:shape,], ;w = 0.5, c=:gray, legend = false, fillalpha = 0.2)
        end
    end

    for obs_idx = 1:1:size(obs_setting, 1)
        h = plot!(h, circleShape(obs_setting[obs_idx][1], obs_setting[obs_idx][2], obs_setting[obs_idx][3]), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 1.0)
    end

    

    return h

end

