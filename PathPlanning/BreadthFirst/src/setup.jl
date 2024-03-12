################################################################################## bfs Setup ##########################################################################################

function defineBFS(
    realsize = [-50; 0; 50; 100],
    mapbound = [51; 51],
    starting_real = [-3; 20],
    ending_real = [50, 20],
    )::BFSSearcher
    bfs = BFSSearcher();
    bfs.s.actualbound = realsize
    bfs.s.mapbound = mapbound
    bfs.s.starting_real = starting_real
    bfs.s.ending_real = ending_real
    bfs.s.x_factor = (bfs.s.actualbound[2] - bfs.s.actualbound[1])/(bfs.s.mapbound[1] - 1)
    bfs.s.y_factor = (bfs.s.actualbound[4] - bfs.s.actualbound[3])/(bfs.s.mapbound[2] - 1)
    bfs.s.starting_pos = [Int(floor((bfs.s.starting_real[1] - bfs.s.actualbound[1])/bfs.s.x_factor + 1)), Int(floor((bfs.s.starting_real[2] - bfs.s.actualbound[3])/bfs.s.y_factor + 1))]
    bfs.s.ending_pos = [Int(floor((bfs.s.ending_real[1] - bfs.s.actualbound[1])/bfs.s.x_factor + 1)), Int(floor((bfs.s.ending_real[2] - bfs.s.actualbound[3])/bfs.s.y_factor + 1))]
    bfs.p.starting_node = BFSNode(nothing, bfs.s.starting_pos)

    bfs.p.nodes_collection[Encode(bfs, bfs.s.starting_pos)] = bfs.p.starting_node
    return bfs
end


function defineBFSobs!(bfs::BFSSearcher, obstacle_list)
    bfs.s.obstacle_list = obstacle_list
    return nothing
end


