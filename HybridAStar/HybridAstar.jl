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

function regulate_states(states, xy_res, ψ_res)
    x = round(states[1]/xy_res)*xy_res
    y = round(states[2]/xy_res)*xy_res
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

minR = 1.0
gear_set =  [1, -1]
steer_set = [-1/minR, 0, 1/minR]
num_steer = size(steer_set, 1)
num_gear = size(gear_set, 1)
T = 1

states_candi, paths_candi = neighbor_origin(T, steer_set, gear_set)

init_st = [0,0,-pi/4]
states_candi = transform(init_st, states_candi)
paths_candi = transform(init_st, paths_candi)

h = plot()
for i = 1:size(paths_candi,3)
    global h
    h = plot!(h, paths_candi[1,:,i], paths_candi[2,:,i], color =:red, legend = false)
    h = scatter!(h, [states_candi[1,i]], [states_candi[2,i]], color =:red, legend = false)
    arrowsize = 0.2
    h = quiver!(h, [states_candi[1,i]], [states_candi[2,i]], quiver = ([arrowsize*cos(states_candi[3,i])],[arrowsize*sin(states_candi[3,i])]) ,color =:red, legend = false)

    xy_res = 0.5
    ψ_res = pi/4
    states_reg = regulate_states(states_candi[:,i], xy_res, ψ_res)


    h = scatter!(h, [states_reg[1]], [states_reg[2]], color =:green, legend = false)
    arrowsize = 0.2
    h = quiver!(h, [states_reg[1]], [states_reg[2]], quiver = ([arrowsize*cos(states_reg[3])],[arrowsize*sin(states_reg[3])]) ,color =:green, legend = false, aspect_ratio =:equal)

end
display(h)