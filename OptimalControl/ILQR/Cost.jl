function TotalCost(statesL, controlL)
    J = 0
    for i = 1:1:size(statesL, 2)-1
        J = J + StageCost(statesL[:, i], controlL[:, i])
    end
    J = J + TerminalCost(statesL[:, end], [0;0])
    return J
end

function StageCost(states, control)
    y       = states[2]
    x       = states[1]
    ux      = states[3]
    ψ       = states[4]
    ax      = control[1]
    δ       = control[2]


    δ_bound_cost = sigmoid_boundary(δ, -pi/6, pi/6)
    ax_bound_cost = sigmoid_boundary(ax, -2, 2)


    J       = 10*ax^2 + 10*δ^2 + 0.01*ux^2 + δ_bound_cost + ax_bound_cost
    # J       = 10 * y^2 + 100 * ψ^2 + 10 * ax^2 + 100 * δ^2
    return J
end
function TerminalCost(states, control)
    x       = states[1]
    y       = states[2]
    ux      = states[3]
    ψ       = states[4]

    terminal_states = [0.0; 0.0; 0.0; 0.0]

    J = 1000*((x - terminal_states[1])^2 + (y - terminal_states[2])^2 + 0.1*(ux - terminal_states[3])^2 + 1*(ψ - terminal_states[4])^2)
    # J       = 0.1*(x - 100) ^ 2
    return J
end



function sigmoid_boundary(st, st_min, st_max)
    slope = 10
    mag   = 100
    st_cost1 = 1/(1 + exp(-slope*(st - st_max)))
    st_cost2 = 1/(1 + exp(slope*(st - st_min)))

    return mag*(st_cost1 + st_cost2)
end

x_max = 4
x_min = -6
x = -10:0.01:10
y = 1 ./ (1 .+ exp.(-10*(x .- x_max)))
y2 = 1 ./ (1 .+ exp.(10*(x .- x_min)))


plot(x, 100*(y2 .+ y))