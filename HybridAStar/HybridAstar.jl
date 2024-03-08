include("../ReedsSheppsCurves/ReedsSheppsCurves.jl")
using DataStructures
using Plots

# alternative solution:
# findall(vertices[:,1].== request_st_code). this is abandoned since it is slower.
function gethandler(st_index_list, st_index)
    # input : 1. the encoded states index(first col of vertices)
    #         2. requested encoded state idx
    # return : handler(Int32) ➡ 0 means doesn't exist, otherwise, return the handler(row idx)
    for i in 1:size(st_index_list,1)
        if Int32(st_index_list[i]) == 0
            return 0
        end
        if Int32(st_index_list[i]) == Int32(st_index)
            return i
        end
    end
    println("Abnormal situation in gethandler: The encoded state idx goes beyond the limit of memory storage")
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

function regulate_states(states,st_reso)
    x_res = st_reso[1]
    y_res = st_reso[2]
    ψ_res = st_reso[3]
    
    x = round(states[1]/xy_res)*x_res
    y = round(states[2]/xy_res)*y_res
    ψ_ori = modπ(states[3])
    ψ = round(ψ_ori/ψ_res)*ψ_res
    return [x,y,ψ]
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


function check_bounds(states, states_bounds)
    for i in 1:3
        if states[i] < states_bounds[i,1] || states[i] > states_bounds[i,2]
            return false
        end
    end
    return true
end


function encode_states(states, states_bounds, resolutions)
    #[xmin xmax; ymin ymax; ψmin ψmax]
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

    return Int32(idx)
end


function decode_states(idx, states_bounds, resolutions)
    x_range = states_bounds[1,:]
    y_range = states_bounds[2,:]
    ψ_range = states_bounds[3,:]

    x_res = resolutions[1]
    y_res = resolutions[2]
    ψ_res = resolutions[3]

    y_num = round((y_range[2] - y_range[1])/y_res) + 1
    ψ_num = round((ψ_range[2] - ψ_range[1])/ψ_res) + 1

    x_id  = floor((idx-1)/(ψ_num*y_num))+1
    res = idx - (x_id - 1)*y_num*ψ_num
    y_id  = floor((res-1)/(ψ_num))+1
    ψ_id  = res - (y_id - 1)*ψ_num

    x = (x_id - 1)*x_res + x_range[1]
    y = (y_id - 1)*y_res + y_range[1]
    ψ = (ψ_id - 1)*ψ_res + ψ_range[1]

    return [x,y,ψ]
end



function block_collision_check(path, block_list)
    block_center_x = block_list[:,1]
    block_center_y = block_list[:,2]
    block_yaw = block_list[:,3]
    block_length = block_list[:,4] .+ 0.5
    block_width = block_list[:,5] .+ 0.5
    p = 4.0

    if size(path, 2) > 25
        x = path[1, 1:25:end]
        y = path[2, 1:25:end]
    else
        x = path[1,1]
        y = path[2,1]
    end

    for i in 1:size(block_list, 1)
        for j in 1:size(x, 1)
            if - ((((cos(block_yaw[i]) * (x[j]-block_center_x[i]) + sin(block_yaw[i]) * (y[j]-block_center_y[i]))/block_length[i] )^p + ( (-sin(block_yaw[i]) * (x[j]-block_center_x[i]) + cos(block_yaw[i]) * (y[j]-block_center_y[i]))/(block_width[i]) )^p + 0.01)^(1/p) ) + 1 >= 0                
                return false
            end
        end
    end
    return true
end


function RS_connected(cur_st, goal_st, minR, block_list)
    norm_states = changeBasis(cur_st, goal_st, minR)
    opt_cmd, opt_cost, _, _ = allpath(norm_states)
    path = createActPath(cur_st, minR, opt_cmd)
    return block_collision_check(path, block_list), path
end

function rs_heuristic(cur_st, goal_st, minR)
    norm_states = changeBasis(cur_st, goal_st, minR)
    _, opt_cost, _, _ = allpath(norm_states)
    return opt_cost*minR
end

function dg_cost(path, block_list)
    #### add obs collision
    if !block_collision_check(path, block_list)
        return Inf
    end
    return 1
end

function validation(st, states_bounds)
    x_range = states_bounds[1,:]
    y_range = states_bounds[2,:]
    ψ_range = states_bounds[3,:]

    x = st[1]
    y = st[2]
    ψ = st[3]

    if x < x_range[1] || x > x_range[2]
        # print("x is :")
        # println(x)
        return false
    end
    if y < y_range[1] || y > y_range[2]
        # print("y is :")
        # println(y)
        return false
    end
    return true
end

############# Define Settings ############# 
minR = 5.0
gear_set =  [1, -1]
steer_set = collect(LinRange(-1/minR,1/minR,7))
# steer_set = [-1/minR, 0, 1/minR]

T = 1.25
states_candi, paths_candi = neighbor_origin(T, steer_set, gear_set)


num_steer = size(steer_set, 1)
num_gear = size(gear_set, 1)
num_states = 3
num_neighbors = Int32(num_steer*num_gear)


xy_res = 0.5
ψ_res = pi/18
st_bounds = [-15 15; -15 5; -pi pi]

st_reso = [xy_res,xy_res,ψ_res]
states_min = regulate_states(st_bounds[:,1], st_reso)
states_max = regulate_states(st_bounds[:,2], st_reso)
st_bounds = [states_min states_max]
############# Define Settings ############# 


############# Define Storage ############# 
maxNum = 30000
# [states_coder, x, y, ψ] here only store states, the cost are stored in a heap for fast min search.
# everytime create a new states, vertices will update one --> the handler of heap map is equal to row idx 
# states_coder == 0 means vertices doesn't exist
vertices = zeros(Int32(maxNum), Int32(num_states + 1)) 

# [parent_handler(means row_idx, not endocded states idx)]
# parent_handler == 0 means no parent
edges = zeros(Int32(maxNum), Int32(1))

# (flag, f, g), flag(visited or not -> be poped or not, 0 means not, 1 means poped, and can be 0 once reinserted) f = g+h, g is traversed cost, h is heuristic cost-to-go
costs_heap = MutableBinaryMinHeap{Tuple{Float64, Float64, Float64}}() 
############# Define Storage ############# 
init_st = [5,-5,pi/2]
goal_st = [-5,-5,pi]
block_list = [-1 -5 0 1 5; -10 -5 0 1.1 3; 5 0 0 2 1; 0 -10 0 10 2] #obs: ox, oy, oψ, l, w


init_st = regulate_states(init_st,st_reso)
goal_st = regulate_states(goal_st,st_reso)

# A star Algorithm started
# step 1. register initial states, and put in the open set (costs_heap)
register_idx = 1

cur_st = init_st
st_index = encode_states(cur_st, st_bounds, st_reso)
vertices[register_idx, 1] = st_index
vertices[register_idx, 2:4] = cur_st 
g_val = 0.0
h_val = rs_heuristic(cur_st, goal_st, minR)
f_val = g_val + h_val
handler = push!(costs_heap, (0.0, f_val, g_val))
if handler != register_idx
    println("Fatal Error: mismatch between cost_heap and vertices")
end


hplot = plot()
# for asd in 1:10
@time begin
while !isempty(costs_heap)

    global block_list, vertices, edges, costs_heap, states_candi, paths_candi,  st_bounds, st_reso, T, num_neighbors, register_idx, hplot
    
    cur_values, handler_cur_idx = top_with_handle(costs_heap)
    cur_st_index = vertices[handler_cur_idx, 1]
    cur_st = vertices[handler_cur_idx, 2:4]
    cur_f = cur_values[2]
    cur_g = cur_values[3]

    termi_flag, final_path = RS_connected(cur_st, goal_st, minR, block_list)
    global final_path
    if termi_flag
        global final_RSpath = final_path
        global final_handler = handler_cur_idx
        break
    end
    # once visited change flag to be visited
    cur_values = (1.0, cur_f, cur_g)
    update!(costs_heap, handler_cur_idx, cur_values)

    neighbors_st = transform(cur_st, states_candi)
    neighbors_path = transform(cur_st, paths_candi)
    hplot = scatter!(hplot, neighbors_st[1,:], neighbors_st[2,:], legend = false, aspect_ratio = :equal)
    
    for neighbor_idx in 1:num_neighbors
        # expand to the neighbors
        nb_st = regulate_states(neighbors_st[:, neighbor_idx], st_reso)   # 3 x 1
        nb_path = neighbors_path[:,:,neighbor_idx] # 3 x N [x;y;ψ]
        if !validation(nb_st, st_bounds)
            continue
        end

        nb_st_index = encode_states(nb_st, st_bounds, st_reso)
        Δg = dg_cost(nb_path, block_list)
        g_temp = cur_g + Δg 

        ######### if the vertice has never been created, then:
        row_idx = gethandler(vertices[:,1], Int32(nb_st_index))
        if row_idx == 0
            register_idx = register_idx + 1
            nb_g = cur_g + Δg
            nb_f = nb_g + rs_heuristic(nb_st, goal_st, minR)

            vertices[register_idx, 1] = nb_st_index
            vertices[register_idx, 2:4] = nb_st 
            edges[register_idx, 1] = handler_cur_idx

            local handler = push!(costs_heap, (0.0, nb_f, nb_g))
            if handler != register_idx
                println("Fatal Error: mismatch between cost_heap and vertices")
            end
        else
            ######### if the vertice exists, then:
            temp_g = cur_g + Δg
            ori_vals = costs_heap[row_idx]
            ori_g = ori_vals[3]
            flag = ori_vals[1]
            
            ######### if the neigh can get lower cost bby choosing cur_st as parent, then:
            if temp_g < ori_g
                edges[row_idx, 1] = handler_cur_idx
                nb_g = temp_g
                nb_f = nb_g + rs_heuristic(nb_st, goal_st, minR)
                update!(costs_heap, row_idx, (0.0, nb_f, nb_g)) # always put in open set
            end
        end
    end
end
end

# h = plot()
# for i = 1:size(paths_candi,3)
#     global h
#     h = plot!(h, paths_candi[1,:,i], paths_candi[2,:,i], color =:red, legend = false)
#     h = scatter!(h, [states_candi[1,i]], [states_candi[2,i]], color =:red, legend = false)
#     arrowsize = 0.2
#     h = quiver!(h, [states_candi[1,i]], [states_candi[2,i]], quiver = ([arrowsize*cos(states_candi[3,i])],[arrowsize*sin(states_candi[3,i])]) ,color =:red, legend = false)

    
#     states_reg = regulate_states(states_candi[:,i], [xy_res, xy_res, ψ_res])


#     h = scatter!(h, [states_reg[1]], [states_reg[2]], color =:green, legend = false)
#     arrowsize = 0.2
#     h = quiver!(h, [states_reg[1]], [states_reg[2]], quiver = ([arrowsize*cos(states_reg[3])],[arrowsize*sin(states_reg[3])]) ,color =:green, legend = false, aspect_ratio =:equal)
# end
# display(h)

h = plot()
# plot the RS part


x_coords = minimum(st_bounds[1,1]):0.1:maximum(st_bounds[1,2])
y_coords = minimum(st_bounds[2,1]):0.1:maximum(st_bounds[2,2])
block_grid = ones(size(y_coords,1), size(x_coords,1))
for i in 1:1:size(x_coords,1)
    for j in 1:1:size(y_coords,1)
        if !block_collision_check([x_coords[i], y_coords[j]], block_list)
            block_grid[j,i] = 0.0
        end
    end
end
h = heatmap(x_coords, y_coords, block_grid, c=:greys, legend = false, colorbar=false)


h = plot!(h, final_path[1,:], final_path[2,:], aspect_ratio =:equal)
# plot hybrid A star part
handler = final_handler
while handler!=0
    global handler = Int32(handler)
    cur_st = vertices[handler, 2:4]
    handler = edges[handler]
    arrowsize = 0.2
    quiver!(h, [cur_st[1]], [cur_st[2]], quiver = ([arrowsize*cos(cur_st[3])], [arrowsize*sin(cur_st[3])]),color =:red, legend = false)
    display(h)
    sleep(0.1)
end