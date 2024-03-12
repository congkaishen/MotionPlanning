

@with_kw mutable struct DKANode
    parent::Union{Int, Nothing}= nothing
    position::Vector{Int64} = Vector{Int64}[]
    f::Float64 = 0
end

Base.isless(a::DKANode, b::DKANode) = isless(a.f, b.f)

@with_kw mutable struct DKASettings
    actualbound::Vector{Float64} = Vector{Float64}[]
    mapbound::Vector{Float64} = Vector{Float64}[]
    starting_pos::Vector{Int64} = Vector{Int64}[]
    ending_pos::Vector{Int64} = Vector{Int64}[]
    starting_real::Vector{Float64} = Vector{Float64}[]
    ending_real::Vector{Float64} = Vector{Float64}[]
    obstacle_list::Vector{Vector{Float64}} = Vector{Vector{Float64}}[]
    x_factor::Float64 = 0.0
    y_factor::Float64 = 0.0
    draw_fig::Bool = true
end

@with_kw mutable struct DKAPlanner
    starting_node::DKANode = DKANode()
    nodes_collection::Dict{Int, DKANode} = Dict{Int, DKANode}()
    open_list::Array{DKANode} = Array{DKANode}[]
    closed_list::Array{DKANode} = Array{DKANode}[]
    loop_count::Int32 = 0
end


@with_kw mutable struct DKAResult
    dkapath = Matrix{Any}[]
    actualpath = Matrix{Any}[]
    planning_time::Float64 = 0.0
end


@with_kw mutable struct DKASearcher
    s::DKASettings = DKASettings()
    p::DKAPlanner = DKAPlanner()
    r::DKAResult = DKAResult()
end
