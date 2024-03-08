#This is enlighted by youtube: Coding a Reeds-Shepp Car Optimal Path Planner
function changeBasis(init_st, termi_st, minR)
    ψ0 = init_st[3]
    ψg = termi_st[3]
    Δx = (termi_st[1] - init_st[1])/minR
    Δy = (termi_st[2] - init_st[2])/minR
    xn = Δx*cos(ψ0) + Δy*sin(ψ0)
    yn = -Δx*sin(ψ0)+ Δy*cos(ψ0)
    ψn = ψg - ψ0
    return [xn, yn, ψn] 
end

function simCarModel(states, ctrls)
    # ctrls = [gear, steer]
    # gear: 1 forward, -1 backward
    # steer: 0 straight, 1 left, -1 right
    ψ =  states[3]
    v =  ctrls[1]
    
    δx = v*cos(ψ)
    δy = v*sin(ψ) 
    δψ = ctrls[2]*v
    return [δx, δy, δψ]
end

function vec2polar(vec)
    r = sqrt(sum(vec.^2))
    θ = atan(vec[2], vec[1])
    return r, θ
end

function modπ(angle)
    if angle >= -pi && angle <= pi
        return angle

    else
        angle = mod(angle, 2*pi)
        if angle < -pi
            angle = angle + 2*pi
        elseif angle > pi
            angle = angle - 2*pi
        end
        return angle
    end
    
end

function path1(states)
#LSL
    x = states[1]
    y = states[2]
    ψ = states[3]
    u, t = vec2polar([x - sin(ψ), y - 1 + cos(ψ)])
    v = modπ(ψ - t)
    cost = abs(t)+abs(u)+abs(v)
    travel = [t,u,v]
    steer = [1,0,1] #1 means left, 0 mean straight, -1 mean right
    gear = [1,1,1]
    cmds = [travel gear steer]
    if (t<0) || (v<0) || (u<0)
        return cmds, Inf
    end
    return cmds,cost
end

function path2(states)
#LSR
    x = states[1]
    y = states[2]
    ψ = states[3]
    ρ, θ = vec2polar([x + sin(ψ), y - 1 - cos(ψ)])
    if ρ >=2
        u = sqrt(ρ^2-4)
        t = modπ(θ + atan(2, u))
        v = modπ(t-ψ)
        cost = abs(t)+abs(u)+abs(v)
        travel = [t,u,v]
        steer = [1,0,-1] #1 means left, 0 mean straight, -1 mean right
        gear = [1,1,1]
        cmds = [travel gear steer]
        if (t<0) || (v<0) || (u<0)
            return cmds, Inf
        else
            return cmds,cost
        end
    else
        return zeros(1,3), Inf
    end
end

function path3(states)
#LRL
    x = states[1]
    y = states[2]
    ψ = states[3]
    ρ, θ = vec2polar([x - sin(ψ), y - 1 + cos(ψ)])
    if ρ<= 4
        a = acos(ρ/4)
        t = modπ(θ + pi/2 + a)
        u = modπ(pi - 2*a)
        v = modπ(ψ-t-u)

        cost = abs(t)+abs(u)+abs(v)
        travel = [t,u,v]
        steer = [1,-1,1] #1 means left, 0 mean straight, -1 mean right
        gear = [1,-1,1] # 1 means forward, -1 means backward
        cmds = [travel gear steer]
        if (t<0) || (v<0) || (u<0)
            return cmds, Inf
        else
            return cmds,cost
        end
    else
        return zeros(1,3), Inf  
    end       
end

function path4(states)
#C|CC, LRL
    x = states[1]
    y = states[2]
    ψ = states[3]

    ρ, θ = vec2polar([x - sin(ψ), y - 1 + cos(ψ)])

    if ρ <= 4
        a = acos(ρ / 4)
        t = modπ(θ + pi/2 + a)
        u = modπ(pi - 2*a)
        v = modπ(t + u - ψ)

        cost = abs(t)+abs(u)+abs(v)
        travel = [t,u,v]
        steer = [1,-1,1] #1 means left, 0 mean straight, -1 mean right
        gear = [1,-1,-1]
        cmds = [travel gear steer]
        if (t<0) || (v<0) || (u<0)
            return  cmds, Inf
        else
            return cmds,cost
        end

    else
        return zeros(1,3), Inf
    end
end

function path5(states)
#CC|C LRL
    x = states[1]
    y = states[2]
    ψ = states[3]

    ρ, θ = vec2polar([x - sin(ψ), y - 1 + cos(ψ)])

    if ρ <= 4
        u = acos(1 - (ρ^2)/8)
        a = asin(2*sin(u)/ρ)
        t = modπ(θ + pi/2 - a)
        v = modπ( t - ψ - u)

        cost = abs(t)+abs(u)+abs(v)
        travel = [t,u,v]
        steer = [1,-1,1] #1 means left, 0 mean straight, -1 mean right
        gear = [1,1,-1]
        cmds = [travel gear steer]
        if (t<0) || (v<0) || (u<0)
            return  cmds, Inf
        else
            return cmds,cost
        end

    else
        return zeros(1,3), Inf
    end
end

function path6(states)
#CCu|CuC, LRLR
    x = states[1]
    y = states[2]
    ψ = states[3]
    ρ, θ = vec2polar([x + sin(ψ), y - 1 - cos(ψ)])
    if ρ <= 4
        if ρ <= 2
            a = acos((ρ + 2) / 4)
            t = modπ(θ + pi/2 + a)
            u = modπ(a)
            v = modπ(ψ - t + 2*u)
        else
            a = acos((ρ - 2) / 4)
            t = modπ(θ + pi/2 - a)
            u = modπ(pi - a)
            v = modπ(ψ - t + 2*u)
        end
        cost = abs(t)+2*abs(u)+abs(v)
        travel = [t,u,u,v]
        steer = [1,-1,1,-1] #1 means left, 0 mean straight, -1 mean right
        gear = [1,1,-1,-1]
        cmds = [travel gear steer]
        if (t<0) || (v<0)|| (u<0)
            return cmds, Inf
        else
            return cmds,cost
        end
    else
        return zeros(1,3), Inf
    end

end

function path7(states)
    #C|CuCu|C LRLR
    x = states[1]
    y = states[2]
    ψ = states[3]
    ρ, θ = vec2polar([x + sin(ψ), y - 1 - cos(ψ)])
    u1 = (20 - ρ^2)/16

    if (ρ <=6) && (0 <= u1) && (u1 <= 1)
        u = acos(u1)
        a = asin(2*sin(u)/ρ)
        t = modπ(θ + pi/2 + a)
        v = modπ(t - ψ)

        cost = abs(t)+2*abs(u)+abs(v)
        travel = [t,u,u,v]
        steer = [1,-1,1,-1] #1 means left, 0 mean straight, -1 mean right
        gear = [1,-1,-1,1]
        cmds = [travel gear steer]

        if (t<0) || (v<0) || (u<0)
            return  cmds, Inf
        else
            return cmds,cost
        end
    else
        return zeros(1,3), Inf
    end
end

function path8(states)
#C|C(π/2)SC LRSL
    x = states[1]
    y = states[2]
    ψ = states[3]
    ρ, θ = vec2polar([x - sin(ψ), y - 1 + cos(ψ)])
    if ρ >= 2
        u = sqrt(ρ^2 - 4) - 2
        a = atan(2, u+2)
        t = modπ(θ + pi/2 + a)
        v = modπ(t - ψ + pi/2)

        cost = abs(t)+ pi/2+abs(u)+abs(v)
        travel = [t,pi/2,u,v]
        steer = [1,-1,0,1] #1 means left, 0 mean straight, -1 mean right
        gear = [1,-1,-1,-1]
        cmds = [travel gear steer]

        if (t<0) || (v<0) || (u<0)
            return  cmds, Inf
        else
            return cmds,cost
        end
    else
        return zeros(1,3), Inf
    end
end

function path9(states)
#CS|C(π/2)C LSRL
    x = states[1]
    y = states[2]
    ψ = states[3]
    ρ, θ = vec2polar([x - sin(ψ), y - 1 + cos(ψ)])
    if ρ>=2
        u = sqrt(ρ^2 - 4)-2
        a = atan(u + 2,2)
        t = modπ(θ + pi/2 - a)
        v = modπ(t - ψ - pi/2)

        cost = abs(t)+ abs(u) + pi/2 +abs(v)
        travel = [t,u,pi/2,v]
        steer = [1,0,-1,1] #1 means left, 0 mean straight, -1 mean right
        gear = [1,1,1,-1]
        cmds = [travel gear steer]

        if (t<0) || (v<0) || (u<0)
            return  cmds, Inf
        else
            return cmds,cost
        end
    else
        return zeros(1,3), Inf
    end
end

function path10(states)
#C|C(π/2)SC LRSR
    x = states[1]
    y = states[2]
    ψ = states[3]
    ρ, θ = vec2polar([x + sin(ψ), y - 1 - cos(ψ)])
    if ρ >=2
        t = modπ(θ + pi/2)
        u = ρ - 2
        v = modπ(ψ - t - pi/2)

        cost = abs(t)+ abs(u) + pi/2 +abs(v)
        travel = [t,pi/2,u,v]
        steer = [1,-1,0,-1] #1 means left, 0 mean straight, -1 mean right
        gear = [1,-1,-1,-1]
        cmds = [travel gear steer]

        if (t<0) || (v<0) || (u<0)
            return  cmds, Inf
        else
            return cmds,cost
        end

    else
        return zeros(1,3), Inf
    end
end

function path11(states)
# CSC(π/2)|C LSLR
    x = states[1]
    y = states[2]
    ψ = states[3]
    ρ, θ = vec2polar([x + sin(ψ), y - 1 - cos(ψ)])
    if ρ >=2
        t = modπ(θ)
        u = ρ - 2
        v = modπ(ψ - t - pi/2)

        cost = abs(t)+ abs(u) + pi/2 +abs(v)
        travel = [t,u,pi/2,v]
        steer = [1,0,1,-1] #1 means left, 0 mean straight, -1 mean right
        gear = [1,1,1,-1]
        cmds = [travel gear steer]

        if (t<0) || (v<0) || (u<0)
            return  cmds, Inf
        else
            return cmds,cost
        end
    else
        return zeros(1,3), Inf
    end

end

function path12(states)
# C|C(π/2)SC(π/2)|C LRSLR
    x = states[1]
    y = states[2]
    ψ = states[3]
    ρ, θ = vec2polar([x + sin(ψ), y - 1 - cos(ψ)])
    if ρ >=4
        u = sqrt(ρ^2-4)-4
        a = atan(2, u+4)
        t = modπ(θ + pi/2 + a)
        v = modπ(t - ψ)

        cost = abs(t)+ pi/2 + abs(u) + pi/2 +abs(v)
        travel = [t,pi/2,u,pi/2,v]
        steer = [1,-1,0,1,-1] #1 means left, 0 mean straight, -1 mean right
        gear = [1,-1,-1,-1,1]
        cmds = [travel gear steer]

        if (t<0) || (v<0) || (u<0)
            return  cmds, Inf
        else
            return cmds,cost
        end
    else
        return zeros(1,3), Inf
    end
end


function reflect(fcn, states)
    states_reflect = deepcopy(states)
    states_reflect[2] = -states_reflect[2]
    states_reflect[3] = -states_reflect[3]    
    cmds, cost = fcn(states_reflect)
    cmds[:,3] = -1*cmds[:,3]
    return cmds, cost
end

function timeflip(fcn, states)
    states_timeflip = deepcopy(states)
    states_timeflip[1] = -states_timeflip[1]
    states_timeflip[3] = -states_timeflip[3]
    cmds, cost = fcn(states_timeflip)
    cmds[:,2] = -1*cmds[:,2]
    return cmds, cost
end

function reverse(fcn, states)
    states_reverse = deepcopy(states)
    states_reverse[1] = -states_reverse[1]
    states_reverse[2] = -states_reverse[2]
    cmds, cost = fcn(states_reverse)
    cmds[:,2] = -1*cmds[:,2]
    cmds[:,3] = -1*cmds[:,3]
    return cmds, cost
end

function createPath(cmds)
    states = [0.0, 0.0, 0.0]
    path = states
    steps = 100
    for i in 1:size(cmds, 1)
        if cmds[i, 2] == 0
            break
        end
        Δt = abs(cmds[i,1])/steps
        for k = 1:steps
            ctrls = cmds[i, 2:3]
            dstates = simCarModel(states, ctrls)            
            states = states + dstates*Δt
            path = vcat(path, states)
        end
    end
    return reshape(path, 3,:)
end


function createActPath(init_st, minR, cmds)
    states = vec(init_st)
    path = states
    steps = 100
    for i in 1:size(cmds, 1)
        if cmds[i, 2] == 0
            break
        end
        Δt = abs(cmds[i,1])/steps
        for k = 1:steps
            ctrls = cmds[i, 2:3]
            dstates = simCarModel(states, ctrls)  
            dstates[1:2] = dstates[1:2]*minR          
            states = states + dstates*Δt
            path = vcat(path, states)
        end
    end
    return reshape(path, 3,:)
end

function allpath(states)
    res_cmds = zeros(5,3,48)
    res_cost = zeros(48)
    fcn_list = [path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12]

    for fcn_idx = 1:1:12
        cmds, cost = fcn_list[fcn_idx](states)
        row = size(cmds, 1)
        col = size(cmds, 2)
        if cost <Inf
            res_cmds[1:row,1:col, (fcn_idx-1)*4+1] = cmds 
        end
        res_cost[(fcn_idx-1)*4+1] = cost

        cmds, cost = timeflip(fcn_list[fcn_idx], states)
        row = size(cmds, 1)
        col = size(cmds, 2)
        if cost <Inf
            res_cmds[1:row,1:col, (fcn_idx-1)*4+2] = cmds 
        end
        res_cost[(fcn_idx-1)*4+2] = cost

        cmds, cost = reflect(fcn_list[fcn_idx], states)
        row = size(cmds, 1)
        col = size(cmds, 2)
        if cost<Inf
            res_cmds[1:row,1:col, (fcn_idx-1)*4+3] = cmds 
        end
        res_cost[(fcn_idx-1)*4+3] = cost

        cmds, cost = reverse(fcn_list[fcn_idx], states)
        row = size(cmds, 1)
        col = size(cmds, 2)
        if cost <Inf
            res_cmds[1:row,1:col, (fcn_idx-1)*4+4] = cmds 
        end
        res_cost[(fcn_idx-1)*4+4] = cost
    end

    min_idx = argmin(res_cost)
    opt_ctrl = res_cmds[:,:,min_idx]
    opt_cost = res_cost[min_idx]
    return opt_ctrl, opt_cost, res_cmds, res_cost
end


function plotActpaths(init_st, minR, cmds, costs)
    fea_idx = findall(costs.<Inf)
    h = plot()

    for idx in fea_idx
        path = createActPath(init_st, minR, cmds[:,:,idx])
        h = plot!(h, path[1,:],path[2,:], aspect_ratio =:equal, color =:gray, alpha = 0.2)
    end 

    min_idx = argmin(costs)
    opt_ctrl = cmds[:,:,min_idx]
    path = createActPath(init_st, minR, opt_ctrl)
    h = plot!(h, path[1,:],path[2,:], aspect_ratio =:equal, color =:blue, alpha = 1.0)
    h = scatter!(h, [path[1,1]],[path[2,1]], color=:green, legend = false)
    h = scatter!(h, [path[1,end]],[path[2,end]], color=:red, legend = false)
    return h
end

function plotpaths(cmds, costs)
    fea_idx = findall(costs.<Inf)
    h = plot()

    for idx in fea_idx
        path = createPath(cmds[:,:,idx])
        h = plot!(h, path[1,:],path[2,:], aspect_ratio =:equal, color =:gray, alpha = 0.2)
    end 

    min_idx = argmin(costs)
    opt_ctrl = cmds[:,:,min_idx]
    path = createPath(opt_ctrl)
    h = plot!(h, path[1,:],path[2,:], aspect_ratio =:equal, color =:blue, alpha = 1.0)
    h = scatter!(h, [path[1,1]],[path[2,1]], color=:green, legend = false)
    h = scatter!(h, [path[1,end]],[path[2,end]], color=:red, legend = false)

    return h
end




