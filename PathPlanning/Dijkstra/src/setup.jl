################################################################################## DKA Setup ##########################################################################################

function defineDKA(
    realsize = [-50; 0; 50; 100],
    mapbound = [51; 51],
    starting_real = [-3; 20],
    ending_real = [50, 20],
    draw_fig = false,
    make_gif = false,
    )::DKASearcher
    DKA = DKASearcher();
    DKA.s.actualbound = realsize
    DKA.s.mapbound = mapbound
    DKA.s.starting_real = starting_real
    DKA.s.ending_real = ending_real
    DKA.s.x_factor = (DKA.s.actualbound[2] - DKA.s.actualbound[1])/(DKA.s.mapbound[1] - 1)
    DKA.s.y_factor = (DKA.s.actualbound[4] - DKA.s.actualbound[3])/(DKA.s.mapbound[2] - 1)
    DKA.s.starting_pos = [Int(floor((DKA.s.starting_real[1] - DKA.s.actualbound[1])/DKA.s.x_factor + 1)), Int(floor((DKA.s.starting_real[2] - DKA.s.actualbound[3])/DKA.s.y_factor + 1))]
    DKA.s.ending_pos = [Int(floor((DKA.s.ending_real[1] - DKA.s.actualbound[1])/DKA.s.x_factor + 1)), Int(floor((DKA.s.ending_real[2] - DKA.s.actualbound[3])/DKA.s.y_factor + 1))]
    DKA.p.starting_node = DKANode(nothing, DKA.s.starting_pos,0)
    DKA.s.draw_fig = draw_fig
    DKA.s.make_gif = make_gif
    DKA.p.nodes_collection[Encode(DKA, DKA.s.starting_pos)] = DKA.p.starting_node
    return DKA
end


function defineDKAobs!(DKA::DKASearcher, obstacle_list)
    DKA.s.obstacle_list = obstacle_list
    return nothing
end


