function circleShape(h,k,r)
    θ = LinRange(0, 2*π, 500)
    h.+r*sin.(θ), k.+r*cos.(θ)
end


function plotRes(dka)


	goal_pt = dka.s.ending_pos
    obs_setting = dka.s.obstacle_list

	h = plot(size = [1000, 600])

    if size(dka.r.actualpath, 1) > 2


        for (key, node) in dka.p.nodes_collection

            act_pos = TransferCoordinate(dka, node.position)

            h = scatter!(h, [act_pos[1]], [act_pos[2]], c=:gray, fillalpha = 0.2)
            # h = plot!(h, circleShape(act_pos[1], act_pos[2], 0.25), seriestype = [:shape,], ;w = 0.5, c=:gray, legend = false, fillalpha = 0.2)
        end

        h = plot!(h, dka.r.actualpath[:,1], dka.r.actualpath[:,2],aspect_ratio=:equal, lc=:red, legend=false)
    end

    for obs_idx = 1:1:size(obs_setting, 1)
        h = plot!(h, circleShape(obs_setting[obs_idx][1], obs_setting[obs_idx][2], obs_setting[obs_idx][3]), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 1.0)
    end
	h = plot!(h, circleShape(goal_pt[1], goal_pt[2], 0.25), seriestype = [:shape,], ;w = 0.5, c=:green, linecolor = :green, legend = false, fillalpha = 1.0)

    

    return h

end



function retrievepath(dka::DKASearcher, ori_current_node::DKANode) 
    dkapath = ori_current_node.position
    if ori_current_node.parent == nothing
        dka.r.actualpath = TransferCoordinate(dka, ori_current_node.position)
        return nothing
    end

    current_node = dka.p.nodes_collection[ori_current_node.parent]

    while current_node != nothing
        dkapath = [dkapath current_node.position]
        if current_node.parent == nothing
            current_node = nothing
        else
            current_node = dka.p.nodes_collection[current_node.parent]
        end
    end
    
    dkapath = [reverse!(dkapath[1, :]) reverse!(dkapath[2, :])]
    dka.r.dkapath = dkapath
    actualpath = deepcopy(dkapath)
    actualpath = convert(Matrix{Float64}, actualpath)
    for i = 1:size(actualpath, 1)
        position_val = actualpath[i, :]
        actualpath[i,:] = TransferCoordinate(dka, position_val)
    end
    dka.r.actualpath  = actualpath
end

function planDKA!(dka::DKASearcher)
    t1 = time()
    push!(dka.p.open_list, dka.p.starting_node)
    while !isempty(dka.p.open_list)
        dka.p.loop_count = dka.p.loop_count + 1
        sort!(dka.p.open_list)
        current_node = popfirst!(dka.p.open_list)

        if dka.s.draw_fig == true && mod(dka.p.loop_count, 100)==1
            retrievepath(dka, current_node) 
            # println(size(dka.r.actualpath))
            h = plotRes(dka)
            display(h)
            sleep(0.001)
        end



        if current_node.position == dka.s.ending_pos
            dkapath = dka.s.ending_pos
            current_node = dka.p.nodes_collection[current_node.parent]
            while current_node != nothing
                dkapath = [dkapath current_node.position]
                if current_node.parent == nothing
                    current_node = nothing
                else
                    current_node = dka.p.nodes_collection[current_node.parent]
                end
            end
            
            dkapath = [reverse!(dkapath[1, :]) reverse!(dkapath[2, :])]
            dka.r.dkapath = dkapath
            actualpath = deepcopy(dkapath)
            actualpath = convert(Matrix{Float64}, actualpath)
            for i = 1:size(actualpath, 1)
                position_val = actualpath[i, :]
                actualpath[i,:] = TransferCoordinate(dka, position_val)
            end
            dka.r.actualpath  = actualpath
            @goto escape_label
        end

        FindNewNode(dka, current_node)
    end


    @label escape_label
    t2 = time()
    dka.r.planning_time = t2 - t1
    return nothing
end



function InOpen(dka::DKASearcher, current_node::DKANode)
    for node in dka.p.open_list
        if current_node.position == node.position
            return true
        end
    end
    return false
end


function InClose(dka::DKASearcher, current_node::DKANode)
    for node in dka.p.closed_list
        if current_node.position == node.position
            return true
        end
    end
    return false
end

function RemoveClose(dka::DKASearcher, current_node::DKANode)
    idx = 0

    for node in dka.p.closed_list
        idx = idx+1
        if current_node.position == node.position
            deleteat!(dka.p.closed_list, Int64(idx))
        end
    end
end


function Encode(dka::DKASearcher, pos::Vector{Int64})
    x = pos[1]
    y = pos[2]
    XMAX = dka.s.mapbound[1]
    YMAX = dka.s.mapbound[2]

    if x<1 || x>XMAX || y<1 || y>YMAX
        return Int64(0)
    else
        dka_idx = (y-1)*XMAX + x
        return Int64(floor(dka_idx))
    end
end

function FindNewNode(dka::DKASearcher, current_node::DKANode)
    cur_idx = Encode(dka, current_node.position)
    directions = [(-1,0),(-1,-1), (-1,-1), (1,-1), (1,0),(1,1), (0,-1), (1,-1), (0,1)]
    # directions = [(-1,0),(1,0),(0,-1), (0,1)]
    for direction in directions
        new_position = [ Int64(current_node.position[1] + direction[1]), Int64(current_node.position[2] + direction[2])]

        neighbor_idx = Encode(dka, new_position)
        need_update = false

        if checkPositionValidity(dka, new_position)
            if neighbor_idx == 0
                continue
            end
            temp_f = current_node.f + sqrt((new_position[1] - current_node.position[1])^2*dka.s.x_factor^2 + (new_position[2] - current_node.position[2])^2*dka.s.y_factor^2 )

            if haskey(dka.p.nodes_collection, neighbor_idx)
                temp_node = deepcopy(dka.p.nodes_collection[ Encode(dka, new_position) ])
                if temp_f<temp_node.f
                    need_update = true
                else
                    need_update = false
                end
            else
                temp_node = DKANode(cur_idx, new_position, temp_f)
                temp_node.f = temp_f
                temp_node.parent = cur_idx
                dka.p.nodes_collection[neighbor_idx] = temp_node
                need_update = true
            end

            if need_update
                temp_node.f = temp_f
                temp_node.parent = cur_idx
                dka.p.nodes_collection[neighbor_idx] = temp_node

                if !InOpen(dka, temp_node)
                     push!(dka.p.open_list, temp_node)
                end

            end


        end

    end
    push!(dka.p.closed_list, current_node)
end

function TransferCoordinate(dka::DKASearcher, position_val)
    new_position = [ (position_val[1] - 1.0) * dka.s.x_factor + dka.s.actualbound[1]; (position_val[2] - 1.0) * dka.s.y_factor + dka.s.actualbound[3]]
    return new_position
end


function checkPositionValidity(dka::DKASearcher, position_val)
    flag = true
    if position_val[1] < 1 || position_val[2] < 1 || position_val[1] > dka.s.mapbound[1] || position_val[2] > dka.s.mapbound[2]
        flag = false
    end
    obsflag = checkObsSafety(dka, position_val)
    return flag && obsflag
end


function checkObsSafety(dka, position_val)
    flag = true
    transferred_position_val = TransferCoordinate(dka, position_val)
    for i = 1:size(dka.s.obstacle_list, 1)
        if (transferred_position_val[1] - dka.s.obstacle_list[i][1])^2 + (transferred_position_val[2] - dka.s.obstacle_list[i][2])^2 < dka.s.obstacle_list[i][3]^2
            flag = false
            break;
        end
    end
    return flag
end
