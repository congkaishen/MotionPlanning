function kinematic(states, ctrls, veh_param)
    L = veh_param[1]
    # states: x, y, ψ
    # ctrls: ux, sa
    ψ = states[3]
    ux = ctrls[1]
    sa = ctrls[2]

    δx = ux*cos(ψ)
    δy = ux*sin(ψ)
    δψ = ux/L*tan(sa)
    return [δx, δy, δψ]
end

function inverseKinematic(cur_st, dstates, veh_param)
    L = veh_param[1]
    δx = dstates[1]
    δy = dstates[2]
    δψ = dstates[3]
    ψ = cur_st[3]

    if abs(cos(ψ)) >= sqrt(2)/2
        ux = δx/cos(ψ)
    else
        ux = δy/sin(ψ)
    end

    if abs(ux) >= 0.01
        sa =  atan((δψ/ux)*L)
    else
        sa = 0
    end
    return [ux, sa]
end

function findclosest(cur_pt, ref_pts, least_idx, maximum_idx)
    pts = ref_pts .- [cur_pt[1] cur_pt[2]]
    dist = sum(pts.^2, dims = 2)
    dist = dist[least_idx:maximum_idx]
    return argmin(dist)[1] + least_idx -1
end



############ The main begins from here ############ 

# You can set plan_path to be false, so that you can just focus on tracking
# BUT!!!, you have to set it as true in the first run to get a reference path to play with
plan_path = true
###########################################
if plan_path
    include("main_hybrid_astar.jl")
else
    try
        ismissing(hybrid_astar)
    catch
        println("Need to plan a path first!")
        println("Please set plan_path to be true")
        sleep(1)
        exit()
    end

    if  ismissing(hybrid_astar)
        println("Need to plan a path first!")
        println("Please set plan_path to be true")
        sleep(1)
        exit()
    else
        if size(hybrid_astar.r.actualpath, 1) <= 1
            println("Can't find a path")
            sleep(1)
            exit()
        end
    end
end


cd(@__DIR__)
############ Assume the main has already run, you have the hybrid_astar planned ############ 
refined_length = LinRange(0, hybrid_astar.r.tol_length, 1000)
# in hybrid A* speed is assumed to be normalized to 1
x_ref = hybrid_astar.r.x_interp(refined_length)
y_ref = hybrid_astar.r.y_interp(refined_length)
ψ_ref = hybrid_astar.r.ψ_interp(refined_length)
ref_pts = [x_ref y_ref]


veh_length = vehicle_size[1]
veh_width = vehicle_size[2]
max_sa = max_δf + 0.1 # let the maximum steering a little bit larger than the planner(or do the opposite in planning for the safety margin)
veh_param = [veh_length, veh_width, max_sa]


############ Tune for feedback PI controller ############ 
look_ahead_dist = 0.5
p_gain = 5
i_gain = 0.2
# p_gain = 0
# i_gain = 0

############ Variable for simulation, DON'T CHANGE IT ############ 
cur_states = starting_real
states_his = cur_states

dt_sim = 1e-3
least_idx = 1
least_look_idx = 1
termi_flag = false

simulation_idx = 0
err_accumulated = 0
while true
    global cur_states, states_his,dt_sim, least_idx, least_look_idx, look_ahead_dist, veh_param, simulation_idx, err_accumulated

    simulation_idx = simulation_idx + 1

    maximum_idx = argmin( abs.(refined_length .- (simulation_idx*dt_sim + look_ahead_dist))   )
    idx = findclosest(cur_states[1:2], ref_pts, least_idx, maximum_idx)
    if idx == maximum(size(ref_pts))
        break
    end
    least_idx = max(idx, argmin( abs.(refined_length .- (simulation_idx*dt_sim))   )) 
    ref_cur = [x_ref[idx], y_ref[idx], ψ_ref[idx]]
    ref_next = [x_ref[idx+1], y_ref[idx+1], ψ_ref[idx+1]]
    dref = (ref_next - ref_cur)/((refined_length[idx+1] - refined_length[idx]+ 1e-4)/1)

    ctrls = inverseKinematic([x_ref[idx], y_ref[idx], ψ_ref[idx]], dref, veh_param)

    look_pt = [look_ahead_dist*cos(cur_states[3]), look_ahead_dist*sin(cur_states[3])]
    if ctrls[1] > 0
        look_pt = cur_states[1:2]+look_pt
    else
        look_pt = cur_states[1:2]-look_pt
    end

    maximum_look_idx = argmin( abs.(refined_length .- (simulation_idx*dt_sim + look_ahead_dist*2))   )
    look_idx = findclosest(look_pt, ref_pts, least_look_idx, maximum_look_idx)
    least_look_idx = max(look_idx, argmin( abs.(refined_length .- (simulation_idx*dt_sim + look_ahead_dist))   ))

    ref_look_pt = [x_ref[look_idx], y_ref[look_idx]]
    
    vec1 = [cos(ψ_ref[look_idx]), sin(ψ_ref[look_idx])]
    vec2 = look_pt - ref_look_pt
    vec1 = [vec1; 0]
    vec2 = [vec2; 0]
    err = cross(vec1, vec2)[3]
    err_accumulated = err_accumulated + err*dt_sim


    ctrls[2] = ctrls[2] + p_gain*(-err) + i_gain*(-err_accumulated)
    ctrls[2] = min(max(ctrls[2], -veh_param[3]), veh_param[3])

    cur_states = cur_states + kinematic(cur_states, ctrls, veh_param)*dt_sim
    states_his = [states_his cur_states]
    
    if mod(simulation_idx, 100) == 0
        title_string = "look-ahead err: $(round(err;digits = 2)) m"
        h = plotPlannerPath(hybrid_astar)
        plot!(h, states_his[1,:], states_his[2,:], color =:black)

        h = PlotVehicleDetailed(h, [cur_states[1], cur_states[2], cur_states[3], veh_param[1]/2, veh_param[2]/2], ctrls[2])

        scatter!(h, [cur_states[1]], [cur_states[2]], color =:red)
        scatter!(h, [ref_cur[1]], [ref_cur[2]], color =:red, alpha = 0.5)

        scatter!(h, [look_pt[1]], [look_pt[2]], color =:blue)
        scatter!(h, [ref_look_pt[1]], [ref_look_pt[2]], color =:blue, alpha = 0.5, title = title_string)

        display(h)
        sleep(0.01)
    end
end



