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

function PlotVehicleDetailed(h, veh_block, sa)
    
    veh_ψ = veh_block[3]
    veh_l = veh_block[4]
    veh_w = veh_block[5]

    veh_x = veh_block[1] + veh_l/2*cos(veh_ψ)
    veh_y = veh_block[2] + veh_l/2*sin(veh_ψ)
    veh_boundary = GetRectanglePts([veh_x, veh_y, veh_ψ, veh_l/2, veh_w/2])
    tire_w = 0.1
    tire_l = 0.2
    # in Get rectangle 3=>LowerRight(right tire), 4=>UpperRight(left tire)
    tire_left_block = [veh_boundary[1, 4], veh_boundary[2, 4], veh_ψ+sa, tire_l, tire_w ]
    tire_left_boundary = GetRectanglePts(tire_left_block)
    tire_right_block = [veh_boundary[1, 3], veh_boundary[2, 3], veh_ψ+sa, tire_l, tire_w ]
    tire_right_boundary = GetRectanglePts(tire_right_block)
    # in Get rectangle 1=>UpperLeft(rear left tire), 2=>Lowerleft(rear right tire)
    tire_rleft_block = [veh_boundary[1, 1], veh_boundary[2, 1], veh_ψ, tire_l, tire_w ]
    tire_rleft_boundary = GetRectanglePts(tire_rleft_block)
    tire_rright_block = [veh_boundary[1, 2], veh_boundary[2, 2], veh_ψ, tire_l, tire_w ]
    tire_rright_boundary = GetRectanglePts(tire_rright_block)

    triangle = [veh_boundary[:,1] veh_boundary[:,2] (veh_boundary[:,3] .+ veh_boundary[:,4])./2 veh_boundary[:,5] ] 
    

    h = plot!(h, veh_boundary[1,:], veh_boundary[2,:], linestyle=:dash, color =:green,  legend = false)
    h = plot!(h, tire_left_boundary[1,:], tire_left_boundary[2,:], seriestype = [:shape,], color =:gray,  legend = false)
    h = plot!(h, tire_right_boundary[1,:], tire_right_boundary[2,:], seriestype = [:shape,], color =:gray,  legend = false)
    h = plot!(h, tire_rleft_boundary[1,:], tire_rleft_boundary[2,:], seriestype = [:shape,], color =:black,  legend = false)
    h = plot!(h, tire_rright_boundary[1,:], tire_rright_boundary[2,:], seriestype = [:shape,], color =:black,  legend = false)
    h = plot!(h, triangle[1,:], triangle[2,:], seriestype = [:shape,], color =:yellow,  legend = false)
    h = plot!(h, veh_boundary[1, 3:4], veh_boundary[2, 3:4], color =:black, linewidth = 3,  legend = false)
    h = plot!(h, veh_boundary[1, 1:2], veh_boundary[2, 1:2], color =:black, linewidth = 3,  legend = false)

    return h
end

function plotTrackingRes(hybrid_astar::HybridAstarSearcher, states_his, cur_states, ctrls, look_pt, ref_cur, ref_look_pt)
    title_string = "Tracking"
    h = plotPlannerPath(hybrid_astar)
    plot!(h, states_his[1,:], states_his[2,:], color =:black)

    h = PlotVehicleDetailed(h, [cur_states[1], cur_states[2], cur_states[3], veh_param[1], veh_param[2]], ctrls[2])

    scatter!(h, [cur_states[1]], [cur_states[2]], color =:red)
    scatter!(h, [ref_cur[1]], [ref_cur[2]], color =:red, alpha = 0.5, title = title_string)

    scatter!(h, [look_pt[1]], [look_pt[2]], color =:blue)
    scatter!(h, [ref_look_pt[1]], [ref_look_pt[2]], color =:blue, alpha = 0.5, title = title_string)
    return h
end