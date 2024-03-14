

@with_kw mutable struct BFSNode
    parent::Union{Int, Nothing}= nothing
    position::Vector{Int64} = Vector{Int64}[]
end


@with_kw mutable struct BFSSetting
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
    make_gif::Bool = false
end

@with_kw mutable struct BFSPlanner
    starting_node::BFSNode = BFSNode()
    nodes_collection::Dict{Int, BFSNode} = Dict{Int, BFSNode}()
    open_list::Array{BFSNode} = Array{BFSNode}[]
    closed_list::Array{BFSNode} = Array{BFSNode}[]
    loop_count::Int32 = 0
end


@with_kw mutable struct BFSResult
    bfspath = Matrix{Any}[]
    actualpath = Matrix{Any}[]
    planning_time::Float64 = 0.0
end


@with_kw mutable struct BFSSearcher
    s::BFSSetting = BFSSetting()
    p::BFSPlanner = BFSPlanner()
    r::BFSResult = BFSResult()
end
