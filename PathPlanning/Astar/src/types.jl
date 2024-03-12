

@with_kw mutable struct AstarNode
    parent::Union{Int, Nothing}= nothing
    position::Vector{Int64} = Vector{Int64}[]
    g::Float64 = 0
    h::Float64 = 0
    f::Float64 = 0
end

Base.isless(a::AstarNode, b::AstarNode) = isless(a.f, b.f)

@with_kw mutable struct AstarSettings
    actualbound::Vector{Float64} = Vector{Float64}[]
    mapbound::Vector{Float64} = Vector{Float64}[]
    starting_pos::Vector{Int64} = Vector{Int64}[]
    ending_pos::Vector{Int64} = Vector{Int64}[]
    starting_real::Vector{Float64} = Vector{Float64}[]
    ending_real::Vector{Float64} = Vector{Float64}[]
    obstacle_list::Vector{Vector{Float64}} = Vector{Vector{Float64}}[]
    x_factor::Float64 = 0.0
    y_factor::Float64 = 0.0
    draw_fig::Bool = false
end

@with_kw mutable struct AstarPlanner
    starting_node::AstarNode = AstarNode()
    nodes_collection::Dict{Int, AstarNode} = Dict{Int, AstarNode}()
    open_list::Array{AstarNode} = Array{AstarNode}[]
    closed_list::Array{AstarNode} = Array{AstarNode}[]
    loop_count::Int32 = 0
end


@with_kw mutable struct AstarResult
    astarpath = Matrix{Any}[]
    actualpath = Matrix{Any}[]
    planning_time::Float64 = 0.0
end


@with_kw mutable struct AstarSearcher
    s::AstarSettings = AstarSettings()
    p::AstarPlanner = AstarPlanner()
    r::AstarResult = AstarResult()
end
