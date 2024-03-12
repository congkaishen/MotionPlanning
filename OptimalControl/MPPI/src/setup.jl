using NearestNeighbors
using Distributions
function defineMPPI(
    numStates::Int = 0,
    numControls::Int = 0,
    lambda::Any = 0.25,
    X0 = fill(NaN,numStates),
    goal = fill(NaN,2), 
    SamplingNumber::Int64 = 500,
    GridNum::Vector{Int64} = [100, 100],
    N::Int64 = 15,
    T::Float64 = 3,
    XL = fill(NaN,numStates),
    XU = fill(NaN,numStates),
    CL = fill(NaN,numControls),
    CU = fill(NaN,numControls),
    )::MPPISearcher
    
    if numControls <= 0
        error("Controls ($numControls) must be > 0")
    end
    if numStates <= 0
        error("States ($numStates) must be > 0")
    end
    if length(X0) != numStates
        error("Length of X0 ($(length(X0))) must match number of states ($numStates)")
    end
    if length(XL) != numStates
        error("Length of XL ($(length(XL))) must match number of states ($numStates)")
    end
    if length(XU) != numStates
        error("Length of XU ($(length(XU))) must match number of states ($numStates)")
    end
    if length(CL) != numControls
        error("Length of CL ($(length(CL))) must match number of controls ($numControls)")
    end
    if length(CU) != numControls
        error("Length of CU ($(length(CU))) must match number of controls ($numControls)")
    end

    MPPI = MPPISearcher()
    MPPI.s.goal = goal
    MPPI.s.numStates = numStates
    MPPI.s.numControls = numControls
    MPPI.s.lambda = lambda
    MPPI.s.X0 = X0
    MPPI.s.XL = XL
    MPPI.s.XU = XU
    MPPI.s.CL = CL
    MPPI.s.CU = CU
    MPPI.s.T = T
    MPPI.s.N = N
    MPPI.s.dt = T/N
    MPPI.s.SamplingNumber = SamplingNumber
    MPPI.s.NominalControl = zeros(MPPI.s.N, MPPI.s.numControls)
    MPPI.s.Σ = Diagonal(ones(numControls)) * 0.1
    MPPI.s.MultivariantNormal = MvNormal(zeros(MPPI.s.numControls), MPPI.s.Σ)
    return MPPI
end

function defineMPPIobs!(MPPI::MPPISearcher, obstacle_list)
    MPPI.s.obstacle_list = obstacle_list
    return nothing
end





function defineMPPINominalControl!(MPPI::MPPISearcher, args...)
    if length(args) == 1
        MPPI.s.NominalControl = args[1]
        return nothing
    else
        MPPI.s.NominalControl = args[1]
        MPPI.s.Σ = args[2]
        MPPI.s.MultivariantNormal = MvNormal(zeros(MPPI.s.numControls), MPPI.s.Σ)
    end
    return nothing
end

