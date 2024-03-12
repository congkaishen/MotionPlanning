struct MyMetrics <: Metric
end

struct CandidacyMetrics <: Metric
end

const g = 9.8

function (dist::MyMetrics)(x, y)
    ψ1 = x[3]
    ψ2 = y[3]
    r = sqrt( sum((y[1:2] .- x[1:2]).^2))+0.1

    if abs(x[1] -y[1]) <= 0.01
        ρ_ψ = pi/2
    else
        ρ_ψ = atan( (x[2] -y[2])/(x[1] -y[1]) )
    end

    Δψ = ψ2-ψ1

    return (40*( 2*(ψ1-ρ_ψ)^2 + 2*(ψ2-ρ_ψ)^2) + 10*((Δψ )^2) + 1*( r*sin( ((ψ1+ψ2)/2  - ρ_ψ)*2  ) )^2)
end

function (dist::CandidacyMetrics)(x, y)
    x1 = x[1]
    y1 = x[2]
    ψ1 = x[3]
    ux1 = x[4]
    dux1 = x[5]

    x2 = y[1]
    y2 = y[2]
    ψ2 = y[3]
    ux2 = y[4]
    dux2 = y[5]

    return sqrt( (x1-x2)^2 + (y1-y2)^2 + 10*(ψ1-ψ2)^2 + 100*(ux1-ux2)^2 + (dux1-dux2)^2)
end




@with_kw mutable struct AstarNode
    parent::Union{Int, Nothing}= nothing
    position::Vector{Int64} = Vector{Int64}[]
    height::Float64 = 0.0
    g::Float64 = 0
    h::Float64 = 0
    f::Float64 = 0
end

Base.isless(a::AstarNode, b::AstarNode) = isless(a.g, b.g)

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
    terrain_kdtree::Any = Any
    terrain_info = Matrix{Any}[]
end

@with_kw mutable struct AstarPlanner
    starting_node::AstarNode = AstarNode()
    nodes_collection::Dict{Int, AstarNode} = Dict{Int, AstarNode}()
    open_list::Array{AstarNode} = Array{AstarNode}[]
    closed_list::Array{AstarNode} = Array{AstarNode}[]
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


@with_kw mutable struct RRTNode
    loc::Vector{Float64} = Vector{Float64}(undef, 2)
    eulerang::Float64 = 0.0
    cost:: Float64 = 0
end

@with_kw mutable struct RRTSettings
    starting_position::Vector{Float64} = Vector{Float64}[]
    ending_position::Vector{Float64} = Vector{Float64}[]
    BoundPosition::Vector{Float64} = Vector{Float64}[]
    obstacle_list::Vector{Vector{Float64}} = Vector{Vector{Float64}}[]
    goal_tolerance::Float64 = 1.0
    grow_dist::Vector{Float64} = [1, 3]
    sampling_number::Int = 0
    buffer_size::Int = 0
    sample_max_dist = 15
    draw_fig::Bool = false
    terrain_kdtree::Any = Any
    terrain_info = Matrix{Any}[]
    candidacy::Any = Any
    candidacytree::Any = Any
    candidate_num::Int = 0
    rrt_star::Bool = false
    use_astar::Bool = false
    bias_rate::Float64 = 0.2
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

@with_kw mutable struct LTRSettings
    starting_position::Vector{Float64} = Vector{Float64}[]
    ending_position::Vector{Float64} = Vector{Float64}[]
    BoundPosition::Vector{Float64} = Vector{Float64}[]
    obstacle_list::Vector{Vector{Float64}} = Vector{Vector{Float64}}[]
    sampling_number::Int = 0
    draw_fig::Bool = false
    terrain_kdtree::Any = Any
    terrain_info = Matrix{Any}[]
    iterations::Int64 = 300
    delta_dist::Float64 = 0.01
    delta_speed::Float64 = 0.01
    delta_yaw::Float64 = 0.05
    LayerInfo::Any = Any
end

@with_kw mutable struct LTRPlanner
    nodes_collection::Array{Float64} = Array{Float64}[]
    costs_collection::Vector{Float64} = Vector{Float64}[]
    states_collection = Matrix{Any}[]
    starting_node::RRTNode = RRTNode()
    ending_node::RRTNode = RRTNode()
    traj::Any = Any
end

@with_kw mutable struct LTRResult
    status::Symbol = :NaN
    Path = Matrix{Any}[]
    tSolve::Float64 = 1e6;
    cost::Float64 = 0;
end


@with_kw mutable struct LTRSearcher
    s::LTRSettings = LTRSettings()
    p::LTRPlanner = LTRPlanner()
    r::LTRResult = LTRResult()
end


########################################################################################## TPP Setting
@with_kw mutable struct TPPResult
    status::Symbol = :NaN
    Path = Matrix{Any}[]
    TrajFollow = Matrix{Any}[]
    tSolve::Float64 = 1e6;
end

@with_kw mutable struct TPPSearcher
    astar::AstarSearcher = AstarSearcher()
    rrt::RRTSearcher = RRTSearcher()
    ltr::LTRSearcher = LTRSearcher()
    r::TPPResult = TPPResult()
end
