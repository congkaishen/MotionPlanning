################################################################################## Astar Setup ##########################################################################################

function defineAstar(
    realsize = [-50; 0; 50; 100],
    mapbound = [51; 51],
    starting_real = [-3; 20],
    ending_real = [50, 20],
    draw_fig = false,
    )::AstarSearcher
    astar = AstarSearcher();
    astar.s.actualbound = realsize
    astar.s.mapbound = mapbound
    astar.s.starting_real = starting_real
    astar.s.ending_real = ending_real
    astar.s.x_factor = (astar.s.actualbound[2] - astar.s.actualbound[1])/(astar.s.mapbound[1] - 1)
    astar.s.y_factor = (astar.s.actualbound[4] - astar.s.actualbound[3])/(astar.s.mapbound[2] - 1)
    astar.s.starting_pos = [Int(floor((astar.s.starting_real[1] - astar.s.actualbound[1])/astar.s.x_factor + 1)), Int(floor((astar.s.starting_real[2] - astar.s.actualbound[3])/astar.s.y_factor + 1))]
    astar.s.ending_pos = [Int(floor((astar.s.ending_real[1] - astar.s.actualbound[1])/astar.s.x_factor + 1)), Int(floor((astar.s.ending_real[2] - astar.s.actualbound[3])/astar.s.y_factor + 1))]
    astar.p.starting_node = AstarNode(nothing, astar.s.starting_pos, 0, 0, 0)
    astar.s.draw_fig = draw_fig
    astar.p.nodes_collection[Encode(astar, astar.s.starting_pos)] = astar.p.starting_node
    return astar
end


function defineAstarobs!(astar::AstarSearcher, obstacle_list)
    astar.s.obstacle_list = obstacle_list
    return nothing
end


