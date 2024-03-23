function dynamics(states, ctrls)
    L = 3;
    δx = ctrls[1]*cos(states[3]);
    δy = ctrls[1]*sin(states[3]);
    δψ = ctrls[1]/L*tan(ctrls[2]);
    return [δx; δy; δψ];
end


function rk4_dynamics(states, ctrls, ΔT)
    k1 = dynamics(states, ctrls)
    k2 = dynamics(states .+ ΔT/2*k1, ctrls)
    k3 = dynamics(states .+ ΔT/2*k2, ctrls)
    k4 = dynamics(states .+ ΔT*k3, ctrls)
    return states .+ ΔT/6*(k1 .+ 2*k2 .+ 2*k3 .+ k4 )
end


function jacobian(states, ctrls)
    ΔT = 1
    ϵ = 1e-3
    A = zeros(size(states, 1),size(states, 1))
    B = zeros(size(states, 1),size(ctrls,1))
    for i in 1:size(states, 1)
        Δstates = zeros(size(states))
        Δstates[i] = ϵ
        f_plus =  rk4_dynamics(states .+ Δstates, ctrls, ΔT)
        f_minus = rk4_dynamics(states .- Δstates, ctrls, ΔT)
        A[:, i] = (f_plus - f_minus)./(2*ϵ)
    end

    for j in 1:size(ctrls, 1)
        Δctrls = zeros(size(ctrls))
        Δctrls[j] = ϵ
        f_plus =  rk4_dynamics(states, ctrls .+ Δctrls, ΔT)
        f_minus = rk4_dynamics(states, ctrls .- Δctrls, ΔT)
        B[:, j] = (f_plus - f_minus)./(2*ϵ)
    end
    return A, B
end


function gradient_hessian(cost_func, states, ctrls)
    ϵ = 1e-3

    lₓ  = zeros(size(states, 1), 1)
    lᵤ  = zeros(size(ctrls, 1), 1)
    lₓₓ = zeros(size(states, 1), size(states, 1))
    lᵤᵤ = zeros(size(ctrls, 1), size(ctrls, 1))
    lᵤₓ = zeros(size(ctrls, 1), size(states, 1))

    for i in 1:size(states, 1)
        Δstates = zeros(size(states))
        Δstates[i] = ϵ
        cost_plus =  cost_func(states .+ Δstates, ctrls)
        cost_minus = cost_func(states .- Δstates, ctrls)
        lₓ[i, 1] = (cost_plus - cost_minus)./(2*ϵ)
    end

    for j in 1:size(ctrls, 1)
        Δctrls = zeros(size(ctrls))
        Δctrls[j] = ϵ
        cost_plus =  cost_func(states, ctrls .+ Δctrls)
        cost_minus = cost_func(states, ctrls .- Δctrls)
        lᵤ[j, 1] = (cost_plus - cost_minus)./(2*ϵ)
    end

    for i in 1:size(states, 1)
        for j in 1:size(states, 1)
            Δi = zeros(size(states))
            Δi[i] = ϵ
            Δj = zeros(size(states))
            Δj[j] = ϵ
            if i == j
                lₓₓ[i,j] = (1/(12*(ϵ^2)))*(-cost_func(states .+ 2*Δi, ctrls) + 16*cost_func(states .+ Δi, ctrls) - 30*cost_func(states, ctrls) + 16*cost_func(states .- Δi, ctrls) - cost_func(states .- 2*Δi, ctrls))
            else
                lₓₓ[i,j] =(1/(4*(ϵ^2)))*(cost_func(states .+ Δi .+ Δj, ctrls) + cost_func(states .- Δi .- Δj, ctrls) - cost_func(states .+ Δi .- Δj, ctrls) - cost_func(states .- Δi .+ Δj, ctrls))
            end
        end
    end

    for i in 1:size(ctrls, 1)
        for j in 1:size(ctrls, 1)
            Δi = zeros(size(ctrls))
            Δi[i] = ϵ
            Δj = zeros(size(ctrls))
            Δj[j] = ϵ
            if i == j
                lᵤᵤ[i,j] = (1/(12*(ϵ^2)))*(-cost_func(states, ctrls .+ 2*Δi) + 16*cost_func(states, ctrls .+ Δi) - 30*cost_func(states, ctrls) + 16*cost_func(states, ctrls .- Δi) - cost_func(states, ctrls .- 2*Δi))
            else
                lᵤᵤ[i,j] =(1/(4*(ϵ^2)))*(cost_func(states, ctrls .+ Δi .+ Δj) + cost_func(states, ctrls .- Δi .- Δj) - cost_func(states, ctrls .+ Δi .- Δj) - cost_func(states, ctrls .- Δi .+ Δj))
            end
        end
    end

    for i in 1:size(ctrls, 1)
        for j in 1:size(states, 1)
            Δi = zeros(size(ctrls))
            Δi[i] = ϵ
            Δj = zeros(size(states))
            Δj[j] = ϵ

            lᵤₓ[i,j] =(1/(4*(ϵ^2)))*(cost_func(states .+ Δj, ctrls .+ Δi ) + cost_func(states .- Δj, ctrls .- Δi ) - cost_func(states .- Δj, ctrls .+ Δi) - cost_func(states .+ Δj, ctrls .- Δi))
        end
    end

    return lₓ, lᵤ, lₓₓ, lᵤᵤ, lᵤₓ
end


function stage_cost(states, ctrls)
    x = states[1]
    y = states[2]
    ψ = states[3]

    ux = ctrls[1]
    sa = ctrls[2]


    obs = [20.0, 0.0, 2.0]

    obs_cost = obs[3]^2 - ((x - obs[1])^2 + (y - obs[2])^2) 

    β = 1
    obs_cost2 = (1/β)*log(1+exp(β*obs_cost))

    return y^2 + ψ^2 + ux^2 + sa^2 + obs_cost
end

function terminal_cost(states, ctrls)
    x = states[1]
    y = states[2]
    ψ = states[3]

    ux = ctrls[1]
    sa = ctrls[2]

    return (x- 30)^2
end


