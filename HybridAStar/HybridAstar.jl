include("../ReedsSheppsCurves/ReedsSheppsCurves.jl")
using Plots

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
    ori_states = deepcopy(states)
    θ = init_st[3]
    x = init_st[1]
    y = init_st[2]
    if size(ori_states, 3) == 1
        if size(ori_states, 2) == 1
            states[1] = ori_states[1]*cos(θ) - ori_states[2]*sin(θ)
            states[2] = ori_states[1]*sin(θ) + ori_states[2]*cos(θ)
            states[3] = ori_states[3] + θ
        else
            states[1,:] = ori_states[1,:]*cos(θ) .- ori_states[2,:]*sin(θ)
            states[2,:] = ori_states[1,:]*sin(θ) .+ ori_states[2,:]*cos(θ)
            states[3,:] = ori_states[3,:] .+ θ
        end
    else
        states[1,:,:] = ori_states[1,:,:]*cos(θ) .- ori_states[2,:,:]*sin(θ)
        states[2,:,:] = ori_states[1,:,:]*sin(θ) .+ ori_states[2,:,:]*cos(θ)
        states[3,:,:] = ori_states[3,:,:] .+ θ
    end

    return states
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


############# Define Settings ############# 
minR = 1.0
gear_set =  [1, -1]
steer_set = [-1/minR, 0, 1/minR]
T = 1

num_steer = size(steer_set, 1)
num_gear = size(gear_set, 1)
states_candi, paths_candi = neighbor_origin(T, steer_set, gear_set)

xy_res = 0.5
ψ_res = pi/4
st_bounds = [-50 50; -50 50; -pi pi]

st_reso = [xy_res,xy_res,ψ_res]
states_min = regulate_states(st_bounds[:,1], st_reso)
states_max = regulate_states(st_bounds[:,2], st_reso)
st_bounds = [states_min states_max]
############# Define Settings ############# 


############# Define Storage ############# 











############# Define Storage ############# 

init_st = [0,0,-pi/4]
states_candi = transform(init_st, states_candi)
paths_candi = transform(init_st, paths_candi)









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