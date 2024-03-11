function update_map(MPPI::MPPISearcher)
    x_coords = MPPI.s.XL[1] : MPPI.s.GridSize[1] : MPPI.s.XU[1]
    y_coords = MPPI.s.XL[2] : MPPI.s.GridSize[2] : MPPI.s.XU[2]
    x_mesh = repeat(x_coords', length(y_coords), 1)
    y_mesh = repeat(y_coords, 1, length(x_coords))
    
    mask = sqrt.((x_mesh .- MPPI.s.X0[1]).^2 .+ (y_mesh .- MPPI.s.X0[2]).^2) .<= MPPI.s.lidar_range

    for occlusion in MPPI.s.occlusions
        angle_min, angle_max = occlusion
        angle = atan.(y_mesh .- MPPI.s.X0[2], x_mesh .- MPPI.s.X0[1])
        mask = mask .& ((angle .<= angle_min + MPPI.s.X0[5]) .| (angle .>= angle_max + MPPI.s.X0[5]))
    end

    MPPI.s.Grid .|= mask
    return nothing
end

function InitialBlindSplotDetection(MPPI::MPPISearcher)
    # This function assumes vehicle position X 0, Y 0, psi 0
    x_coords = -MPPI.s.lidar_range : MPPI.s.GridSize[1] : MPPI.s.lidar_range
    y_coords =-MPPI.s.lidar_range : MPPI.s.GridSize[2] : MPPI.s.lidar_range
    y_mesh = repeat(y_coords', length(x_coords), 1)
    x_mesh = repeat(x_coords, 1, length(y_coords))
    mask = BitMatrix(ones(size(x_mesh)))
    for occlusion in MPPI.s.occlusions
        angle_min, angle_max = occlusion
        angle = atan.(y_mesh .- MPPI.s.X0[2], x_mesh .- MPPI.s.X0[1])
        mask = mask .& ((angle .<= angle_min ) .| (angle .>= angle_max))
    end

    blind_idxes = findall(mask .== false)
    vis_idxes = findall(mask .== true)

    BlindCord = zeros(size(blind_idxes, 1), 2)
    VisCord   = zeros(size(vis_idxes, 1), 2)

    count = 1
    for idx in blind_idxes
        BlindCord[count ,:] = [x_mesh[idx], y_mesh[idx]]
        count += 1
    end

    count = 1
    for idx in vis_idxes
        VisCord[count ,:] = [x_mesh[idx], y_mesh[idx]]
        count += 1
    end

    MPPI.s.VisCord = VisCord
    MPPI.s.BlindCord = BlindCord
    return nothing
end

