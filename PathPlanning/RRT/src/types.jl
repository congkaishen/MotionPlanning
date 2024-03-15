struct MyMetrics <: Metric
end


const g = 9.8

function (dist::MyMetrics)(x, y)
    r = sqrt( sum((y[1:2] .- x[1:2]).^2))
    return r
end



@with_kw mutable struct RRTNode
    loc::Vector{Float64} = Vector{Float64}(undef, 2)
    cost:: Float64 = 0
end

@with_kw mutable struct RRTSettings
    starting_position::Vector{Float64} = Vector{Float64}[]
    ending_position::Vector{Float64} = Vector{Float64}[]
    BoundPosition::Vector{Float64} = Vector{Float64}[]
    obstacle_list::Vector{Vector{Float64}} = Vector{Vector{Float64}}[]
    goal_tolerance::Float64 = 1.0
    grow_dist::Vector{Float64} = [0, 3]
    sampling_number::Int = 0
    buffer_size::Int = 0
    sample_max_dist = 15
    terrain_kdtree::Any = Any
    terrain_info = Matrix{Any}[]
    candidacy::Any = Any
    candidacytree::Any = Any
    candidate_num::Int = 0
    rrt_star::Bool = false
    use_astar::Bool = false
    bias_rate::Float64 = 0.2
    draw_fig::Bool = false
    make_gif::Bool = false
end

@with_kw mutable struct RRTPlanner
    nodes_collection::Dict{Int, RRTNode} = Dict{Int, RRTNode}()
    costs_collection::Vector{Float64} = Vector{Float64}[]
    parents_collection::Dict{Int,Int} = Dict{Int, Int}()
    children_collection::Dict{Int, Set{Int}} = Dict{Int, Set{Int}}()
    states_collection = Matrix{Any}[]
    buffer_idx::Int = 0
    starting_node::RRTNode = RRTNode()
    ending_node::RRTNode = RRTNode()
    balltree::Any = Any
    kdtree::Any = Any
    sample_idx::Int = 1
    sampling_bias_points::Any = Any
    loop_count::Int32 = 0
end

@with_kw mutable struct RRTResult
    status::Symbol = :NaN
    Path = Matrix{Any}[]
    resultIdxs::Vector{Int64} = Vector{Int64}[]
    tSolve::Float64 = 1e6;
    cost::Float64 = 0;
end


@with_kw mutable struct RRTSearcher
    s::RRTSettings = RRTSettings()
    p::RRTPlanner = RRTPlanner()
    r::RRTResult = RRTResult()
end


# rrt settings ends here

######################################################################################## TPP Settings #######################################################################################


