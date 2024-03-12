function planAstar!(astar::AstarSearcher)
    t1 = time()
    push!(astar.p.open_list, astar.p.starting_node)
    while !isempty(astar.p.open_list)
        astar.p.loop_count = astar.p.loop_count + 1
        sort!(astar.p.open_list)
        current_node = popfirst!(astar.p.open_list)

        if current_node.position == astar.s.ending_pos
            astarpath = astar.s.ending_pos
            current_node = astar.p.nodes_collection[current_node.parent]
            while current_node != nothing
                astarpath = [astarpath current_node.position]
                if current_node.parent == nothing
                    current_node = nothing
                else
                    current_node = astar.p.nodes_collection[current_node.parent]
                end
            end
            
            astarpath = [reverse!(astarpath[1, :]) reverse!(astarpath[2, :])]
            astar.r.astarpath = astarpath
            actualpath = deepcopy(astarpath)
            actualpath = convert(Matrix{Float64}, actualpath)
            for i = 1:size(actualpath, 1)
                position_val = actualpath[i, :]
                actualpath[i,:] = TransferCoordinate(astar, position_val)
            end
            astar.r.actualpath  = actualpath
            @goto escape_label
        end

        FindNewNode(astar, current_node)
    end


    @label escape_label
    t2 = time()
    astar.r.planning_time = t2 - t1
    return nothing
end



function InOpen(astar::AstarSearcher, current_node::AstarNode)
    for node in astar.p.open_list
        if current_node.position == node.position
            return true
        end
    end
    return false
end


function InClose(astar::AstarSearcher, current_node::AstarNode)
    for node in astar.p.closed_list
        if current_node.position == node.position
            return true
        end
    end
    return false
end

function RemoveClose(astar::AstarSearcher, current_node::AstarNode)
    idx = 0

    for node in astar.p.closed_list
        idx = idx+1
        if current_node.position == node.position
            deleteat!(astar.p.closed_list, Int64(idx))
        end
    end
end


function Encode(astar::AstarSearcher, pos::Vector{Int64})
    x = pos[1]
    y = pos[2]
    XMAX = astar.s.mapbound[1]
    YMAX = astar.s.mapbound[2]

    if x<1 || x>XMAX || y<1 || y>YMAX
        return Int64(0)
    else
        astar_idx = (y-1)*XMAX + x
        return Int64(floor(astar_idx))
    end
end

function FindNewNode(astar::AstarSearcher, current_node::AstarNode)
    cur_idx = Encode(astar, current_node.position)
    directions = [(-1,0),(-1,-1), (-1,-1), (1,-1), (1,0),(1,1), (0,-1), (1,-1), (0,1)]
    # directions = [(-1,0),(1,0),(0,-1), (0,1)]
    for direction in directions
        new_position = [ Int64(current_node.position[1] + direction[1]), Int64(current_node.position[2] + direction[2])]

        neighbor_idx = Encode(astar, new_position)
        need_update = false

        if checkPositionValidity(astar, new_position)
            if neighbor_idx == 0
                continue
            end
            temp_g =current_node.g + sqrt((new_position[1] - current_node.position[1])^2*astar.s.x_factor^2 + (new_position[2] - current_node.position[2])^2*astar.s.y_factor^2 )
            temp_h = sqrt((new_position[1] - astar.s.ending_pos[1])^2*astar.s.x_factor^2 + (new_position[2] - astar.s.ending_pos[2])^2*astar.s.y_factor^2)
            temp_f = temp_g + temp_h

            if haskey(astar.p.nodes_collection, neighbor_idx)
                temp_node = deepcopy(astar.p.nodes_collection[ Encode(astar, new_position) ])
                if temp_g<temp_node.g
                    need_update = true
                else
                    need_update = false
                end
            else
                temp_node = AstarNode(cur_idx, new_position, temp_g, temp_h, temp_f)
                temp_node.g = temp_g
                temp_node.h = temp_h
                temp_node.f = temp_f
                temp_node.parent = cur_idx
                astar.p.nodes_collection[neighbor_idx] = temp_node
                need_update = true
            end

            if need_update
                temp_node.g = temp_g
                temp_node.h = temp_h
                temp_node.f = temp_f
                temp_node.parent = cur_idx
                astar.p.nodes_collection[neighbor_idx] = temp_node

                if !InOpen(astar, temp_node)
                     push!(astar.p.open_list, temp_node)
                end

            end




            # if InOpen(astar, temp_node)
            #     if !need_update
            #         continue
            #     end
            # elseif InClose(astar, temp_node)
            #     if !need_update
            #         continue
            #     end
            #
            #     RemoveClose(astar, temp_node)
            #     temp_node.f = temp_f
            #     push!(astar.p.open_list, temp_node)
            # else
            #     temp_node.g = temp_g
            #     temp_node.h = temp_h
            #     temp_node.f = temp_f
            #     temp_node.parent = cur_idx
            #     astar.p.nodes_collection[neighbor_idx] = temp_node
            # end


        end

    end
    push!(astar.p.closed_list, current_node)
end

function TransferCoordinate(astar::AstarSearcher, position_val)
    new_position = [ (position_val[1] - 1.0) * astar.s.x_factor + astar.s.actualbound[1]; (position_val[2] - 1.0) * astar.s.y_factor + astar.s.actualbound[3]]
    return new_position
end


function checkPositionValidity(astar::AstarSearcher, position_val)
    flag = true
    if position_val[1] < 1 || position_val[2] < 1 || position_val[1] > astar.s.mapbound[1] || position_val[2] > astar.s.mapbound[2]
        flag = false
    end
    obsflag = checkObsSafety(astar, position_val)
    return flag && obsflag
end


function checkObsSafety(astar, position_val)
    flag = true
    transferred_position_val = TransferCoordinate(astar, position_val)
    for i = 1:size(astar.s.obstacle_list, 1)
        if (transferred_position_val[1] - astar.s.obstacle_list[i][1])^2 + (transferred_position_val[2] - astar.s.obstacle_list[i][2])^2 < astar.s.obstacle_list[i][3]^2
            flag = false
            break;
        end
    end
    return flag
end
