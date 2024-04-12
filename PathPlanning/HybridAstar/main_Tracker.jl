include("src/hybrid_astar_utils.jl")
include("src/tracker_utils.jl")

############ The main begins from here ############ 

# You can set plan_path to be false, so that you can just focus on tracking
# BUT!!!, you have to set it as true in the first run to get a reference path to play with
plan_path = false
tr_draw_fig = true
tr_make_gif = true

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
look_ahead_dist = 1.0
p_gain = 10
i_gain = 0.1
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

if tr_make_gif anim = Plots.Animation() end

while true
    global cur_states, states_his,dt_sim, least_idx, least_look_idx, look_ahead_dist, veh_param, simulation_idx, err_accumulated, anim

    simulation_idx = simulation_idx + 1

    maximum_idx = argmin( abs.(refined_length .- (simulation_idx*dt_sim + look_ahead_dist))   )
    idx = findclosest(cur_states[1:2], ref_pts, least_idx, maximum_idx)
    if idx == maximum(size(ref_pts))
        h = plotTrackingRes(hybrid_astar, states_his, cur_states, ctrls, look_pt, ref_cur, ref_look_pt)
        display(h)
        sleep(1)
        break
    end
    least_idx = max(idx, argmin( abs.(refined_length .- (simulation_idx*dt_sim))   )) 
    global ref_cur = [x_ref[idx], y_ref[idx], ψ_ref[idx]]
    ref_next = [x_ref[idx+1], y_ref[idx+1], ψ_ref[idx+1]]
    dref = (ref_next - ref_cur)/((refined_length[idx+1] - refined_length[idx]+ 1e-4)/1)

    global ctrls = inverseKinematic([x_ref[idx], y_ref[idx], ψ_ref[idx]], dref, veh_param)

    global look_pt = [look_ahead_dist*cos(cur_states[3]), look_ahead_dist*sin(cur_states[3])]
    if ctrls[1] > 0
        look_pt = cur_states[1:2]+look_pt
    else
        look_pt = cur_states[1:2]-look_pt
    end

    maximum_look_idx = argmin( abs.(refined_length .- (simulation_idx*dt_sim + look_ahead_dist*2))   )
    look_idx = findclosest(look_pt, ref_pts, least_look_idx, maximum_look_idx)
    least_look_idx = max(look_idx, argmin( abs.(refined_length .- (simulation_idx*dt_sim + look_ahead_dist))   ))

    global ref_look_pt = [x_ref[look_idx], y_ref[look_idx]]
    
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
    
    if (tr_draw_fig || tr_make_gif) && (mod(simulation_idx, 100) == 0)
        h = plotTrackingRes(hybrid_astar, states_his, cur_states, ctrls, look_pt, ref_cur, ref_look_pt)

        if tr_draw_fig
            display(h)
            sleep(0.01)
        end

        if tr_make_gif
            Plots.frame(anim)
        end

    end
end


if tr_make_gif 
    gif(anim, "./gifholder/tracker.gif", fps = 10) 
end

