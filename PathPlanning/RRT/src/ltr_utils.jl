function InitialCheckLTR(states)
    Δx = states[1]
    Δy = states[2]
    Δψ_head = states[3]
    u1 = states[4]
    u2 = states[5]

    if Δx<0.8 || Δx>4.0
        return false
    end

    if Δy<-1.5 || Δy>1.5
        return false
    end

    if Δψ_head<-0.36 || Δψ_head>0.36
        return false
    end

    if u1<1 || u1>9
        return false
    end

    if u2<1 || u2>9 || abs(u2-u1) >5
        return false
    end

    return true
end


function CalculateLTRCost(ltr::LTRSearcher)
    cost = 0;
    for i = 1:1:size(ltr.p.nodes_collection, 1) - 1
        start_node = ltr.p.nodes_collection[i, :]
        next_node = ltr.p.nodes_collection[i + 1, :]
        proj_states = ProjectionStates([start_node next_node])
        cost = cost + neural_cost(proj_states)
    end
    ltr.r.cost = cost
end

function RRT2LTR(tpp::TPPSearcher)
    final_idxs = reverse(tpp.rrt.r.resultIdxs)
    traj = zeros(size(final_idxs, 1), 8)
    for i = 1:1:size(final_idxs, 1)
        traj[i, :] = vcat(tpp.rrt.p.nodes_collection[final_idxs[i]].loc[:], tpp.rrt.p.nodes_collection[final_idxs[i]].eulerang,tpp.rrt.p.nodes_collection[final_idxs[i]].ux,tpp.rrt.p.nodes_collection[final_idxs[i]].cost)
    end
    tpp.ltr.p.traj = traj
    PortingLTRTraj!(tpp.ltr, tpp.rrt.s.BoundPosition)
    return nothing
end

function CheckInitialTraj(ltr::LTRSearcher)
    flag = true
    for i = 2:1:size(ltr.p.traj, 1)-1
        prev_node = (ltr.p.nodes_collection[i-1, :])
        new_node =  (ltr.p.nodes_collection[i, :])
        if (collision(ltr.s.obstacle_list, [prev_node new_node], ltr))
            flag = false
            return flag
        end
    end
    return flag
end


# function executeLTRArray(ltr::LTRSearcher)
#     t1 = time()
#     iterations = ltr.s.iterations
#     delta_dist = ltr.s.delta_dist
#     delta_speed = ltr.s.delta_speed
#     delta_yaw = ltr.s.delta_yaw
#     dist_dircs = [-1,0,1]
#     speed_dircs = [-1,0,1]
#     yaw_dircs = [-1,0,1]
#     p = Progress(iterations, 0.5, "LTR Progress...", 60)

#     idx_list = 2:1:ltr.s.sampling_number
#     for iter = 1:1:iterations
#         idx_list = shuffle(idx_list)
#         # delta_speed = rand(1)[1]*(0.5-0.01)+0.01
#         for idx_idx = 1:1:ltr.s.sampling_number-1
#             idx = idx_list[idx_idx]
#             temp_states = zeros(8, 2, 27)
#             if idx == ltr.s.sampling_number
#                 for i  = 1:1:3
#                     for j = 1:1:3
#                         for k = 1:1:3
#                             query_node = ltr.p.nodes_collection[idx, :]
#                             prev_node = ltr.p.nodes_collection[idx-1, :]
#                             query_R = euler2Rot(query_node[4:6])
#                             lateral_vec = query_R[:, 2]

#                             dist_dirc = 0
#                             speed_dirc = speed_dircs[j]
#                             yaw_dirc = yaw_dircs[k]

#                             newloc = query_node[1:3] + dist_dirc*delta_dist*lateral_vec
#                             newux = query_node[7] + speed_dirc*delta_speed
#                             ψ = query_node[6] + yaw_dirc*delta_yaw

#                             idxs, dists = knn(ltr.s.terrain_kdtree, newloc[1:2], 5, true)
#                             newloc[3] = ltr.s.terrain_info[3, idxs[1]]
#                             normalvec = getNormalVec(ltr, newloc)
#                             neweulerang = query_node[4:6]

#                             θ = asin(normalvec[1])
#                             φ = asin(-normalvec[2]/(cos(θ)))

#                             neweulerang[1] = φ
#                             neweulerang[2] = θ
#                             neweulerang[3] = ψ
#                             new_node = [newloc; neweulerang; newux; 0.0]

#                             proj_states = ProjectionStates([prev_node new_node])
#                             new_node[8] = prev_node[8] + neural_cost(proj_states)
#                             temp_states[:, 1, 3*((i-1)*3+j-1)+k ] = new_node

#                             temp_next_node = new_node
#                             temp_next_node[8] = new_node[8]
#                             temp_states[:, 2, 3*((i-1)*3+j-1)+k] = temp_next_node
#                         end
#                     end
#                 end

#                 ori_temp_states = temp_states
#                 min_idx = argmin(temp_states[8, 2, :])
#                 new_node = ori_temp_states[:, 1, min_idx]
#                 temp_next_node = ori_temp_states[:, 2, min_idx]
#                 search_count = 1

#                 while (collision(ltr.s.obstacle_list, [ltr.p.nodes_collection[idx-1, :] new_node], ltr)) && (search_count <= 27)
#                     temp_states[8, 2, min_idx] = temp_states[8, 2, min_idx] + 1000000
#                     min_idx = argmin(temp_states[8, 2, :])
#                     new_node = ori_temp_states[:, 1, min_idx]
#                     temp_next_node = ori_temp_states[:, 2, min_idx]
#                     search_count = search_count + 1
#                 end

#                 if search_count == 28
#                     min_idx = 14
#                 end

#                 new_node = ori_temp_states[:, 1, min_idx]
#                 temp_next_node = ori_temp_states[:, 2, min_idx]

#                 ltr.p.nodes_collection[idx, :] = new_node
#                 ltr.p.states_collection[idx,:] = [new_node[1:3];new_node[6];new_node[7]]
#             else

#                 for i  = 1:1:3
#                     for j = 1:1:3
#                         for k = 1:1:3
#                             query_node = ltr.p.nodes_collection[idx, :]
#                             next_node = ltr.p.nodes_collection[idx+1, :]
#                             prev_node = ltr.p.nodes_collection[idx-1, :]
#                             query_R = euler2Rot(query_node[4:6])
#                             lateral_vec = query_R[:, 2]

#                             dist_dirc = dist_dircs[i]
#                             speed_dirc = speed_dircs[j]
#                             yaw_dirc = yaw_dircs[k]

#                             newloc = query_node[1:3] + dist_dirc*delta_dist*lateral_vec
#                             newux = query_node[7] + speed_dirc*delta_speed
#                             ψ = query_node[6] + yaw_dirc*delta_yaw

#                             idxs, dists = knn(ltr.s.terrain_kdtree, newloc[1:2], 5, true)
#                             newloc[3] = ltr.s.terrain_info[3, idxs[1]]
#                             normalvec = getNormalVec(ltr, newloc)
#                             neweulerang = query_node[4:6]

#                             θ = asin(normalvec[1])
#                             φ = asin(-normalvec[2]/(cos(θ)))

#                             neweulerang[1] = φ
#                             neweulerang[2] = θ
#                             neweulerang[3] = ψ
#                             new_node = [newloc; neweulerang; newux; 0.0]

#                             proj_states = ProjectionStates([prev_node new_node])
#                             new_node[8] = prev_node[8] + neural_cost(proj_states)
#                             temp_states[:, 1, 3*((i-1)*3+j-1)+k ] = new_node

#                             temp_next_node = next_node
#                             proj_states = ProjectionStates([new_node temp_next_node])
#                             temp_next_node[8] = new_node[8] + neural_cost(proj_states)
#                             temp_states[:, 2, 3*((i-1)*3+j-1)+k] = temp_next_node
#                         end
#                     end
#                 end

#                 ori_temp_states = temp_states
#                 min_idx = argmin(temp_states[8, 2, :])
#                 new_node = ori_temp_states[:, 1, min_idx]
#                 temp_next_node = ori_temp_states[:, 2, min_idx]
#                 search_count = 1

#                 while (collision(ltr.s.obstacle_list, [ltr.p.nodes_collection[idx-1, :] new_node], ltr) || collision(ltr.s.obstacle_list, [new_node temp_next_node], ltr)) && (search_count <= 27)
#                     temp_states[8, 2, min_idx] = temp_states[8, 2, min_idx] + 1000000
#                     min_idx = argmin(temp_states[8, 2, :])
#                     new_node = ori_temp_states[:, 1, min_idx]
#                     temp_next_node = ori_temp_states[:, 2, min_idx]
#                     search_count = search_count + 1
#                 end

#                 # if min_idx == 14
#                 #     println("no change")
#                 #     println([iter idx search_count])
#                 # end

#                 if search_count == 28
#                     min_idx = 14
#                 end

#                 new_node = ori_temp_states[:, 1, min_idx]
#                 temp_next_node = ori_temp_states[:, 2, min_idx]

#                 ltr.p.nodes_collection[idx, :] = new_node
#                 ltr.p.states_collection[idx,:] = [new_node[1:3];new_node[6];new_node[7]]

#                 ltr.p.nodes_collection[idx+1, :] = temp_next_node
#                 ltr.p.states_collection[idx+1,:] = [temp_next_node[1:3];temp_next_node[6];temp_next_node[7]]
#             end

#         end
#         next!(p)
#         # println(ltr.p.nodes_collection[end, 8])
#     end

#     ltr.r.Path = ltr.p.states_collection
#     t2 = time()
#     ltr.r.status = :Solved
#     ltr.r.tSolve = t2 - t1

#     if ltr.s.draw_fig
#         h = LTRplot(ltr)
#     end
#     return nothing;
# end

###########################################


function executeLTRArray(ltr::LTRSearcher)
    t1 = time()
    iterations = ltr.s.iterations
    delta_dist = ltr.s.delta_dist
    delta_speed = ltr.s.delta_speed
    delta_yaw = ltr.s.delta_yaw
    dist_dircs = [-1,0,1]
    speed_dircs = [-1,0,1]
    yaw_dircs = [-1,0,1]
    p = Progress(iterations, 0.5, "LTR Progress...", 60)

    idx_list = 2:1:ltr.s.sampling_number
    for iter = 1:1:iterations
        idx_list = shuffle(idx_list)
        # delta_speed = rand(1)[1]*(0.5-0.01)+0.01
        for idx_idx = 1:1:ltr.s.sampling_number-1
            idx = idx_list[idx_idx]
            temp_states = zeros(8, 2, 27)
            if idx == ltr.s.sampling_number
                for ijk = 1:1:27

                    k = mod(ijk, 3)
                    if k == 0
                        k = 3
                    end

                    j = Int( (mod(ijk-k, 9))/3 + 1)
                    if j == 0
                        j = 3
                    end
                    i = Int( ((ijk-k)/3 - (j-1))/3 + 1)

                    query_node = ltr.p.nodes_collection[idx, :]
                    prev_node = ltr.p.nodes_collection[idx-1, :]
                    query_R = euler2Rot(query_node[4:6])
                    lateral_vec = query_R[:, 2]

                    dist_dirc = 0
                    speed_dirc = speed_dircs[j]
                    yaw_dirc = yaw_dircs[k]

                    newloc = query_node[1:3] + dist_dirc*delta_dist*lateral_vec
                    newux = query_node[7] + speed_dirc*delta_speed
                    ψ = query_node[6] + yaw_dirc*delta_yaw

                    idxs, dists = knn(ltr.s.terrain_kdtree, newloc[1:2], 5, true)
                    newloc[3] = ltr.s.terrain_info[3, idxs[1]]
                    normalvec = getNormalVec(ltr, newloc, idxs)

                    neweulerang = query_node[4:6]

                    θ = asin(normalvec[1])
                    φ = asin(-normalvec[2]/(cos(θ)))

                    neweulerang[1] = φ
                    neweulerang[2] = θ
                    neweulerang[3] = ψ
                    new_node = [newloc; neweulerang; newux; 0.0]

                    proj_states = ProjectionStates([prev_node new_node])
                    new_node[8] = prev_node[8] + neural_cost(proj_states)
                    temp_states[:, 1, ijk] = new_node

                    temp_next_node = new_node
                    temp_next_node[8] = new_node[8]
                    temp_states[:, 2, ijk] = temp_next_node
                end

                ori_temp_states = temp_states
                min_idx = argmin(temp_states[8, 2, :])
                new_node = ori_temp_states[:, 1, min_idx]
                temp_next_node = ori_temp_states[:, 2, min_idx]
                search_count = 1

                while (collision(ltr.s.obstacle_list, [ltr.p.nodes_collection[idx-1, :] new_node], ltr)) && (search_count <= 27)
                    temp_states[8, 2, min_idx] = temp_states[8, 2, min_idx] + 1000000
                    min_idx = argmin(temp_states[8, 2, :])
                    new_node = ori_temp_states[:, 1, min_idx]
                    temp_next_node = ori_temp_states[:, 2, min_idx]
                    search_count = search_count + 1
                end

                if search_count == 28
                    min_idx = 14
                end

                new_node = ori_temp_states[:, 1, min_idx]
                temp_next_node = ori_temp_states[:, 2, min_idx]

                ltr.p.nodes_collection[idx, :] = new_node
                ltr.p.states_collection[idx,:] = [new_node[1:3];new_node[6];new_node[7]]
            else
                for ijk = 1:1:27

                    k = mod(ijk, 3)
                    if k == 0
                        k = 3
                    end

                    j = Int( (mod(ijk-k, 9))/3 + 1)
                    if j == 0
                        j = 3
                    end
                    i = Int( ((ijk-k)/3 - (j-1))/3 + 1)
                    
                    query_node = ltr.p.nodes_collection[idx, :]
                    next_node = ltr.p.nodes_collection[idx+1, :]
                    prev_node = ltr.p.nodes_collection[idx-1, :]
                    query_R = euler2Rot(query_node[4:6])
                    lateral_vec = query_R[:, 2]

                    dist_dirc = dist_dircs[i]
                    speed_dirc = speed_dircs[j]
                    yaw_dirc = yaw_dircs[k]

                    newloc = query_node[1:3] + dist_dirc*delta_dist*lateral_vec
                    newux = query_node[7] + speed_dirc*delta_speed
                    ψ = query_node[6] + yaw_dirc*delta_yaw

                    idxs, dists = knn(ltr.s.terrain_kdtree, newloc[1:2], 5, true)
                    newloc[3] = ltr.s.terrain_info[3, idxs[1]]
                    normalvec = getNormalVec(ltr, newloc, idxs)


                    neweulerang = query_node[4:6]

                    θ = asin(normalvec[1])
                    φ = asin(-normalvec[2]/(cos(θ)))
                    neweulerang = [φ; θ; ψ]
                    
                    new_node = [newloc; neweulerang; newux; 0.0]

                    proj_states = ProjectionStates([prev_node new_node])
                    new_node[8] = prev_node[8] + neural_cost(proj_states)
                    temp_states[:, 1, ijk ] = new_node

                    temp_next_node = next_node
                    proj_states = ProjectionStates([new_node temp_next_node])
                    temp_next_node[8] = new_node[8] + neural_cost(proj_states)
                    temp_states[:, 2, ijk] = temp_next_node
                end

                ori_temp_states = temp_states
                min_idx = argmin(temp_states[8, 2, :])
                new_node = ori_temp_states[:, 1, min_idx]
                temp_next_node = ori_temp_states[:, 2, min_idx]
                search_count = 1

                while (collision(ltr.s.obstacle_list, [ltr.p.nodes_collection[idx-1, :] new_node], ltr) || collision(ltr.s.obstacle_list, [new_node temp_next_node], ltr)) && (search_count <= 27)
                    temp_states[8, 2, min_idx] = temp_states[8, 2, min_idx] + 1000000
                    min_idx = argmin(temp_states[8, 2, :])
                    new_node = ori_temp_states[:, 1, min_idx]
                    temp_next_node = ori_temp_states[:, 2, min_idx]
                    search_count = search_count + 1
                end

                if min_idx == 14
                    # println("no change")
                    # println([iter idx search_count])
                end

                if search_count == 28
                    min_idx = 14
                end

                new_node = ori_temp_states[:, 1, min_idx]
                temp_next_node = ori_temp_states[:, 2, min_idx]

                ltr.p.nodes_collection[idx, :] = new_node
                ltr.p.states_collection[idx,:] = [new_node[1:3];new_node[6];new_node[7]]

                ltr.p.nodes_collection[idx+1, :] = temp_next_node
                ltr.p.states_collection[idx+1,:] = [temp_next_node[1:3];temp_next_node[6];temp_next_node[7]]
            end

        end
        next!(p)
        # println(ltr.p.nodes_collection[end, 8])
    end

    ltr.r.Path = ltr.p.states_collection
    t2 = time()
    ltr.r.status = :Solved
    ltr.r.tSolve = t2 - t1

    if ltr.s.draw_fig
        h = LTRplot(ltr)
    end
    return nothing;
end