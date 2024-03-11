using Parameters
using LinearAlgebra
@with_kw mutable struct MPPIHolder
    Trajectory                              = Matrix{Float64}[]
    Control                                 = Matrix{Float64}[]
    Feasibility::Bool                       = 0
    cost::Float64                           = 1e6
end
Base.isless(a::MPPIHolder, b::MPPIHolder)       = isless(a.cost, b.cost)
@with_kw mutable struct MPPISetting
    numStates::Int64                        = 0
    numControls::Int64                      = 0
    X0::Vector{Float64}                     = Vector{Float64}[]
    XL::Vector{Float64}                     = Vector{Float64}[]
    XU::Vector{Float64}                     = Vector{Float64}[]
    CL::Vector{Float64}                     = Vector{Float64}[] 
    CU::Vector{Float64}                     = Vector{Float64}[] 
    dt::Float64                             = 0.0
    T::Float64                              = 0.0
    N::Int64                                = 0
    goal::Vector{Float64}                   = Vector{Float64}[]
    obstacle_list::Vector{Vector{Float64}}  = Vector{Vector{Float64}}[]
    GridNum::Vector{Int64}                  = [100, 100]
    GridSize::Vector{Float64}               = [0.0, 0.0]
    XGrid                                   = Matrix{Float32}[]
    YGrid                                   = Matrix{Float32}[]
    TGrid                                   = Matrix{Bool}[] #Touch
    VGrid                                   = Matrix{Bool}[] #Visible
    OGrid                                   = Matrix{Bool}[] #Obstacle
    TempGrid                                = Matrix{Bool}[] #Obstacle
    VisCord                                 = Matrix{Float32}[]
    BlindCord                               = Matrix{Float32}[]
    lidar_range                             = 60
    occlusions                              = [[-pi/8, pi/8], [1.57, 2.00]]
    SamplingNumber::Int64                   = 0
    FeasibilityCount::Int64                 = 900
    tmax::Float64                           = 0.18
    NominalControl                          = Matrix{Float64}[]
    Σ                                       = Matrix{Float64}[]
    lambda::Any                             = 0.02 #Inverse Temperature
    MultivariantNormal::Any                 = Any
    SlackPenalty::Float64                   = 1e5
    MultiThreadBoolean::Bool                = false
end
@with_kw mutable struct MPPIPlanner
    TrajectoryCollection::Vector{MPPIHolder}  = Vector{MPPIHolder}[]
end

@with_kw mutable struct MPPIResult
    Traj                                    = Matrix{Float64}[]
    Control                                 = Matrix{Float64}[]
    Feasibility::Symbol                     = :InFeasible
    cost::Float64                           = 1e6
    time::Float64                           = 0.0
    FeasibleTrajCount::Int64                = 0
    RolloutCount::Int64                     = 0
end

@with_kw mutable struct MPPISearcher
    s::MPPISetting                          = MPPISetting()
    p::MPPIPlanner                          = MPPIPlanner()
    r::MPPIResult                           = MPPIResult()
end