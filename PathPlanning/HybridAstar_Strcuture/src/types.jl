

@with_kw mutable struct HybridAstarNode
    parent::Union{Int, Nothing}= nothing
    states::Vector{Int64} = Vector{Int64}[]
    g::Float64 = 0
    h::Float64 = 0
    f::Float64 = 0
end

Base.isless(a::HybridAstarNode, b::HybridAstarNode) = isless(a.f, b.f)

@with_kw mutable struct HybridAstarSettings
    actualbound::Vector{Float64} = Vector{Float64}[]
    mapbound::Vector{Float64} = Vector{Float64}[]
    starting_states::Vector{Int64} = Vector{Int64}[]
    ending_states::Vector{Int64} = Vector{Int64}[]
    starting_real::Vector{Float64} = Vector{Float64}[]
    ending_real::Vector{Float64} = Vector{Float64}[]
    obstacle_list::Vector{Vector{Float64}} = Vector{Vector{Float64}}[]
    x_factor::Float64 = 0.0
    y_factor::Float64 = 0.0
    draw_fig::Bool = false
end

@with_kw mutable struct HybridAstarPlanner
    starting_node::HybridAstarNode = HybridAstarNode()
    nodes_collection::Dict{Int, HybridAstarNode} = Dict{Int, HybridAstarNode}()
    open_list::Array{HybridAstarNode} = Array{HybridAstarNode}[]
    closed_list::Array{HybridAstarNode} = Array{HybridAstarNode}[]
    loop_count::Int32 = 0
end


@with_kw mutable struct HybridAstarResult
    hybrid_astarpath = Matrix{Any}[]
    actualpath = Matrix{Any}[]
    planning_time::Float64 = 0.0
end


@with_kw mutable struct HybridAstarSearcher
    s::HybridAstarSettings = HybridAstarSettings()
    p::HybridAstarPlanner = HybridAstarPlanner()
    r::HybridAstarResult = HybridAstarResult()
end
