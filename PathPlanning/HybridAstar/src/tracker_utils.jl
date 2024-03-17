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

function plotTrackingRes(hybrid_astar::HybridAstarSearcher, states_his, cur_states, ctrls, look_pt, ref_cur, ref_look_pt)
    title_string = "Tracking"
    h = plotPlannerPath(hybrid_astar)
    plot!(h, states_his[1,:], states_his[2,:], color =:black)

    h = PlotVehicleDetailed(h, [cur_states[1], cur_states[2], cur_states[3], veh_param[1]/2, veh_param[2]/2], ctrls[2])

    scatter!(h, [cur_states[1]], [cur_states[2]], color =:red)
    scatter!(h, [ref_cur[1]], [ref_cur[2]], color =:red, alpha = 0.5)

    scatter!(h, [look_pt[1]], [look_pt[2]], color =:blue)
    scatter!(h, [ref_look_pt[1]], [ref_look_pt[2]], color =:blue, alpha = 0.5, title = title_string)
    return h
end