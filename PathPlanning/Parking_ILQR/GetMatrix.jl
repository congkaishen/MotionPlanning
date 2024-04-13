
######### Numerical Approximation of Central Diff ######### 
function CalculateMatrix(states, ctrls, cost_func)
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

function LocallyLinearizeDynamics(states, ctrls, ΔT)
    ϵ = 1e-3
    A = zeros(size(states, 1),size(states, 1))
    B = zeros(size(states, 1),size(ctrls,1))
    for i in 1:size(states, 1)
        Δstates = zeros(size(states))
        Δstates[i] = ϵ
        f_plus =  RK4Integration(states .+ Δstates, ctrls, ΔT)'
        f_minus = RK4Integration(states .- Δstates, ctrls, ΔT)'

        A[:, i] = (f_plus - f_minus)./(2*ϵ)
    end

    for j in 1:size(ctrls, 1)
        Δctrls = zeros(size(ctrls))
        Δctrls[j] = ϵ
        f_plus =  RK4Integration(states, ctrls .+ Δctrls, ΔT)'
        f_minus = RK4Integration(states, ctrls .- Δctrls, ΔT)'
        B[:, j] = (f_plus - f_minus)./(2*ϵ)
    end
    return A, B
end


######### Using Packages ######### 
# function LocallyLinearizeDynamics(states, control, δT)
#     Ak      = ForwardDiff.jacobian((x) -> RK4Integration(x, control, δT), states)
#     Bk      = ForwardDiff.jacobian((u) -> RK4Integration(states, u, δT), control)
#     return Ak, Bk
# end