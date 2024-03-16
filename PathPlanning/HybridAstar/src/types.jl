
using Parameters

@with_kw mutable struct HybridAstarNode
    parent::Union{Int, Nothing}= nothing
    states::Vector{Float64} = Vector{Float64}[]
    index::Int64 = 0
    g::Float64 = 0
    h::Float64 = 0
    f::Float64 = 0
end
Base.isless(a::HybridAstarNode, b::HybridAstarNode) = isless(a.f, b.f)

@with_kw mutable struct HybridAstarSettings
    starting_states::Vector{Float64} = Vector{Float64}[]
    ending_states::Vector{Float64} = Vector{Float64}[]
    starting_real::Vector{Float64} = Vector{Float64}[]
    ending_real::Vector{Float64} = Vector{Float64}[]
    obstacle_list::Vector{Vector{Float64}} = Vector{Vector{Float64}}[]
    gear_set::Vector{Float64} = Vector{Float64}[]
    steer_set::Vector{Float64} = Vector{Float64}[]
    states_candi = Matrix{Any}[]
    paths_candi = Matrix{Any}[]
    stbound = Matrix{Any}[]
    resolutions::Vector{Float64} = Vector{Float64}[]
    num_steer::Int32 = 0
    num_gear::Int32 = 0
    num_states::Int32 = 0
    num_neighbors::Int32 = 0
    draw_fig::Bool = false
    minR::Float64 = 1.0
    expand_time::Float64 = 1.0
    starting_index::Int64 = 0
end

@with_kw mutable struct HybridAstarPlanner
    starting_node::HybridAstarNode = HybridAstarNode()
    nodes_collection::Dict{Int, HybridAstarNode} = Dict{Int, HybridAstarNode}()
    open_list::Array{HybridAstarNode} = Array{HybridAstarNode}[]
    closed_list::Array{HybridAstarNode} = Array{HybridAstarNode}[]
    loop_count::Int32 = 0
end


@with_kw mutable struct HybridAstarResult
    hybrid_astar_states = Matrix{Any}[]
    actualpath = Matrix{Any}[]
    planning_time::Float64 = 0.0
end


@with_kw mutable struct HybridAstarSearcher
    s::HybridAstarSettings = HybridAstarSettings()
    p::HybridAstarPlanner = HybridAstarPlanner()
    r::HybridAstarResult = HybridAstarResult()
end
