using Parameters
using LinearAlgebra
@with_kw mutable struct DWAHolder
    Trajectory                              = Matrix{Float64}[]
    Control                                 = Matrix{Float64}[]
    Feasibility::Bool                       = false
    cost::Float64                           = 1e6
end
Base.isless(a::DWAHolder, b::DWAHolder)       = isless(a.cost, b.cost)
@with_kw mutable struct DWASetting
    numStates::Int64                        = 0
    numControls::Int64                      = 0
    X0::Vector{Float64}                     = Vector{Float64}[]
    XL::Vector{Float64}                     = Vector{Float64}[]
    XU::Vector{Float64}                     = Vector{Float64}[]
    CL::Vector{Float64}                     = Vector{Float64}[] 
    CU::Vector{Float64}                     = Vector{Float64}[] 
    dt::Float64                             = 0.0
    T::Float64                              = 0.0
    N::Int64                                = 20
    goal::Vector{Float64}                   = Vector{Float64}[]
    obstacle_list::Vector{Vector{Float64}}  = Vector{Vector{Float64}}[]
    SlackPenalty::Float64                   = 1e5
    SampleNumber::Vector{Int64}             = Vector{Int64}[]
    ControlSamples::Matrix{Float64}         = Matrix{Float64}(undef, 0, 0)
end
@with_kw mutable struct DWAPlanner
    TrajectoryCollection::Vector{DWAHolder}  = Vector{DWAHolder}[]
end

@with_kw mutable struct DWAResult
    Traj                                    = Matrix{Float64}[]
    Control                                 = Matrix{Float64}[]
    Feasibility::Symbol                     = :InFeasible
    cost::Float64                           = 1e6
    time::Float64                           = 0.0
end

@with_kw mutable struct DWASearcher
    s::DWASetting                          = DWASetting()
    p::DWAPlanner                          = DWAPlanner()
    r::DWAResult                           = DWAResult()
end