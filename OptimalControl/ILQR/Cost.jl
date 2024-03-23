function TotalCost(statesL, controlL)
    J = 0
    for i = 1:1:size(statesL, 1)-1
        J = J + StageCost(statesL[i, :], controlL[i, :])
    end
    J = J + TerminalCost(statesL[end, :], [0 0])
    return J
end

function StageCost(states, control)
    y       = states[2]
    x       = states[1]
    ψ       = states[4]
    ax      = control[1]
    δ       = control[2]

    J       = 10 * y^2 + 100 * ψ^2 + 10 * ax^2 + 100 * δ^2
    return J
end
function TerminalCost(states, control)
    x       = states[1]
    J       = 0.1*(x - 100) ^ 2
    return J
end