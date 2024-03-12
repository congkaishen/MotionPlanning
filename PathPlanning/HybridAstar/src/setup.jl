################################################################################## HybridAstar Setup ##########################################################################################

function defineHybridAstar(
    gear_set = [1, -1],
    steer_set = collect(LinRange(-1,1,7)),
    minR = 1.0,
    expand_time = 1.0,
    resolutions = [0.2, 0.2, pi/10],
    stbound = [-15 15; -15 5; -pi pi],
    starting_real = [10.0,-5.0,0.0],
    ending_real = [-5.0,-5.0,0.0],
    draw_fig = false,
    )::HybridAstarSearcher
    hybrid_astar = HybridAstarSearcher();
    hybrid_astar.s.gear_set = gear_set
    hybrid_astar.s.steer_set = steer_set
    
    hybrid_astar.s.num_gear = maximum(size(gear_set))
    hybrid_astar.s.num_steer = maximum(size(steer_set))
    hybrid_astar.s.num_states = hybrid_astar.s.num_gear*hybrid_astar.s.num_steer
    hybrid_astar.s.num_neighbors = hybrid_astar.s.num_states

    hybrid_astar.s.minR = minR
    hybrid_astar.s.expand_time = expand_time
    hybrid_astar.s.resolutions = resolutions
    
    states_min = regulate_states(hybrid_astar, stbound[:,1])
    states_max = regulate_states(hybrid_astar, stbound[:,2])
    hybrid_astar.s.stbound = [states_min states_max]

    hybrid_astar.s.starting_real = starting_real
    hybrid_astar.s.ending_real = ending_real


    hybrid_astar.s.starting_states = regulate_states(hybrid_astar, starting_real)
    hybrid_astar.s.ending_states = regulate_states(hybrid_astar, ending_real)
    st_index = Encode(hybrid_astar, hybrid_astar.s.starting_states)
    
    hybrid_astar.p.starting_node = HybridAstarNode(nothing, hybrid_astar.s.starting_states, st_index, 0, 0, 0)
    hybrid_astar.s.draw_fig = draw_fig

    hybrid_astar.s.states_candi,  hybrid_astar.s.paths_candi = neighbor_origin(expand_time, steer_set, gear_set)

    hybrid_astar.p.nodes_collection[Encode(hybrid_astar, hybrid_astar.s.starting_states)] = hybrid_astar.p.starting_node
    return hybrid_astar
end


function defineHybridAstarobs!(hybrid_astar::HybridAstarSearcher, obstacle_list)
    hybrid_astar.s.obstacle_list = obstacle_list
    return nothing
end


